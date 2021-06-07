//
// Created by jbs on 21. 5. 28..
//
#include <zed2_client/Utils.h>

using namespace zed_client;

/// Calculate average orientation using quaternions
Eigen::Quaternionf zed_client::averageQuaternions(const vector<Eigen::Quaternionf>& quatVec) {


    Eigen::Matrix<float, 4, 4> A = Eigen::Matrix<float, 4, 4>::Zero();
    uint sum(0);
    for (auto it = quatVec.begin(); it != quatVec.end(); ++it) {
        Eigen::Matrix<float, 1, 4> q(1,4);
        q(0) = it->w();
        q(1) = it->x();
        q(2) = it->y();
        q(3) = it->z();
        A += q.transpose()*q;
        sum++;
    }
    A /= sum;

    Eigen::EigenSolver<Eigen::Matrix<float, 4, 4>> es(A);

    Eigen::Matrix<std::complex<float>, 4, 1> mat(es.eigenvalues());
    int index;
    mat.real().maxCoeff(&index);
    Eigen::Matrix<float, 4, 1> largest_ev(es.eigenvectors().real().block(0, index, 4, 1));

    return Eigen::Quaternion<float>(largest_ev(0), largest_ev(1), largest_ev(2), largest_ev(3));
}

Point zed_client::operator*( float scalar,const Point& pnt){
    return  pnt.operator*(scalar);
}

sensor_msgs::ImagePtr zed_client::imageToROSmsg(const cv::Mat& img, const std::string encodingType, std::string frameId, ros::Time t) {
    sensor_msgs::ImagePtr ptr = boost::make_shared<sensor_msgs::Image>();
    sensor_msgs::Image& imgMessage = *ptr;
    imgMessage.header.stamp = t;
    imgMessage.header.frame_id = frameId;
    imgMessage.height = img.rows;
    imgMessage.width = img.cols;
    imgMessage.encoding = encodingType;
    int num = 1; //for endianness detection
    imgMessage.is_bigendian = !(*(char *) &num == 1);
    imgMessage.step = img.cols * img.elemSize();
    size_t size = imgMessage.step * img.rows;
    imgMessage.data.resize(size);

    if (img.isContinuous())
        memcpy((char*) (&imgMessage.data[0]), img.data, size);
    else {
        uchar* opencvData = img.data;
        uchar* rosData = (uchar*) (&imgMessage.data[0]);
        for (unsigned int i = 0; i < img.rows; i++) {
            memcpy(rosData, opencvData, imgMessage.step);
            rosData += imgMessage.step;
            opencvData += img.step;
        }
    }
    return ptr;
}


bool zed_client::pngDecompressDepth(const sensor_msgs::CompressedImageConstPtr &depthPtr, cv::Mat &decompressed) {

    cv::Mat decompressedTemp;
    const size_t split_pos = depthPtr->format.find(';');
    const std::string image_encoding =depthPtr->format.substr(0, split_pos);

    if (depthPtr->data.size() > sizeof(compressed_depth_image_transport::ConfigHeader))
    {
        Timer timer;
        // Read compression type from stream
        compressed_depth_image_transport::ConfigHeader compressionConfig;
        memcpy(&compressionConfig, &depthPtr->data[0], sizeof(compressionConfig));
        const std::vector<uint8_t> imageData(depthPtr->data.begin() + sizeof(compressionConfig),depthPtr->data.end());

        if (enc::bitDepth(image_encoding) == 32)
            try{
                decompressedTemp = cv::imdecode(imageData, cv::IMREAD_UNCHANGED);
            }
            catch (cv::Exception& e){
                ROS_ERROR("%s", e.what());
                return false ;
            }


        size_t rows = decompressedTemp.rows;
        size_t cols = decompressedTemp.cols;


        if ((rows > 0) && (cols > 0)) {
            decompressed = cv::Mat(rows, cols, CV_32FC1);

            // Depth conversion
            auto itDepthImg = decompressed.begin<float>(),
                    itDepthImg_end = decompressed.end<float>();
            auto itInvDepthImg = decompressedTemp.begin<unsigned short>(),
                    itInvDepthImg_end = decompressedTemp.end<unsigned short>();

            float depthQuantA = compressionConfig.depthParam[0];
            float depthQuantB = compressionConfig.depthParam[1];

            for (; (itDepthImg != itDepthImg_end) &&
                   (itInvDepthImg != itInvDepthImg_end); ++itDepthImg, ++itInvDepthImg) {
                // check for NaN & max depth
                if (*itInvDepthImg) {
                    *itDepthImg = depthQuantA / ((float) *itInvDepthImg - depthQuantB);
                } else {
                    *itDepthImg = std::numeric_limits<float>::quiet_NaN();
                }
            }

            double elapseDecomp = timer.stop();
            ROS_DEBUG("depth decomp took %f ms", elapseDecomp);
            return true;
        }else
            return false;
    }else
        return false;
}

bool zed_client::jpegDecompressRgb(const sensor_msgs::CompressedImageConstPtr &rgbPtr, cv::Mat &decompressed) {
    Timer timer;
    try {
        decompressed = cv::imdecode(rgbPtr->data, cv::IMREAD_COLOR);
    }
    catch (cv::Exception& e){
        ROS_ERROR("%s", e.what());
        return false;
    }
    double elapseDecomp = timer.stop();
    ROS_DEBUG("rgb decomp took %f ms",elapseDecomp);
    return true;
}

/**
 * draw valid skeletons
 * @param object
 * @param mat image to be masked
 * @param color mean color of this object (R,G,B)
 * @param patchSize colorCenter +/- (pathSize, patchSize) : color averaging area
 * @return true if at least one skeleton found
 */
void zed_client::insertSkeletonPoint(const zed_interfaces::Object &object, cv::Mat &mat, Eigen::Vector3i& color, int patchSize) {

    // extract keypoint
    const int nInterestPoints = 3;
    int keypointIdx[nInterestPoints] = {1,8,11};
    std::vector<Eigen::Vector2f> keyPoints; // size < nInterestPoints due to nan skeleton
    float centerX = 0.0 ,centerY = 0.0;
    for (int i = 0; i < nInterestPoints ; i++) {
        int idx = keypointIdx[i];
        float x = object.skeleton_2d.keypoints[idx].kp[0];
        float y = object.skeleton_2d.keypoints[idx].kp[1];
        if (x >= 0 ) { // valid keypoints
            keyPoints.push_back(Eigen::Vector2f(x,y));
            centerX += x; centerY += y;
        }
    }
    Eigen::Vector2i colorCenter;
    if (not keyPoints.empty()) {
        centerX /= keyPoints.size();
        centerY /= keyPoints.size();
        colorCenter = Eigen::Vector2i (centerX , centerY);
    }else {
        ROS_WARN("obj %d no skeleton. find color from center", object.label_id);
        float bbCenterX = (object.bounding_box_2d.corners[0].kp[0]  + object.bounding_box_2d.corners[2].kp[0]) / 2.0 ;
        float bbCenterY = (object.bounding_box_2d.corners[0].kp[1]  + object.bounding_box_2d.corners[2].kp[1]) / 2.0 ;
        colorCenter = Eigen::Vector2i(bbCenterX,bbCenterY);
    }
    // compute mean color
    Eigen::Vector2i patchLeft = colorCenter - Eigen::Vector2i (patchSize,patchSize);
    Eigen::Vector2i patchRight = colorCenter + Eigen::Vector2i (patchSize,patchSize);
    int nPixel = 0;
    float colorR =0 ,colorG= 0,colorB =0;
    for (int r = std::max(0, patchLeft.y()); r < std::min(mat.rows, patchRight.y()); r++) {
        for (int c = std::max (0,patchLeft.x()) ; c < std::min (mat.cols,patchRight.x()) ; c++ ) {
            auto ptr_r =  mat.ptr<uchar>(r);
            colorR += ptr_r[3 * c + 2];
            colorG += ptr_r[3 * c + 1];
            colorB += ptr_r[3 * c + 0];
            nPixel++;
        }
    }
    colorR /= nPixel; colorG /= nPixel; colorB /= nPixel;
    color = Eigen::Vector3i(colorR,colorG,colorB);
    for(auto keypoint: keyPoints) {
        cv::circle(mat, cv::Point(keypoint.x(), keypoint.y()), 7,
                   cv::Scalar(0, 0, 0), 3);
    }
}

tf::Vector3 zed_client::vec2Tf(Eigen::Vector3f vec) {return tf::Vector3(vec.x(),vec.y(),vec.z());}
Eigen::Vector3f zed_client::corner2Vec(zed_interfaces::Keypoint3D corner) {
    return Eigen::Vector3f(corner.kp[0],corner.kp[1],corner.kp[2]);
}


/**
 * Transform from object frame to bb origin )
 * x = e1 = 0 (p3 - p0, width) , y = e2 (p4 - p0 , height) , z = e3 (p1 - p0, length)
 * @param object
 * @return
 */
Pose zed_client::extractBbPoseFromSensor(const zed_interfaces::Object &object) {

    zed_interfaces::BoundingBox3D bb = object.bounding_box_3d;
    Eigen::Vector3f p0 = corner2Vec(object.bounding_box_3d.corners[0]);
    Eigen::Vector3f p1 = corner2Vec(object.bounding_box_3d.corners[1]);
    Eigen::Vector3f p4 = corner2Vec(object.bounding_box_3d.corners[4]);
    Eigen::Vector3f p3 = corner2Vec(object.bounding_box_3d.corners[3]);

    // x = e1 = 0 (p3 - p0, width) , y = e2 (p4 - p0 , height) , z = e3 (p1 - p0, length)
    tf::Vector3 e1 = vec2Tf(p3-p0); e1.normalize();
    tf::Vector3 e2 = vec2Tf(p4-p0); e2.normalize();
    tf::Vector3 e3 = vec2Tf(p1-p0); e3.normalize();

    tf::Matrix3x3 rot; // rotation from R_{cam}_{bb}
    rot.setValue(e1.x(),e2.x(),e3.x(),e1.y(),e2.y(),e3.y(),e1.z(),e2.z(),e3.z());
    Pose poseMat;
    tf::Quaternion quat; rot.getRotation(quat); Eigen::Quaternionf quatEigen(quat.w(),quat.x(),quat.y(),quat.z());
    Eigen::Vector3f origin(object.position[0],object.position[1],object.position[2]); // t_{cam}_{bb}
    poseMat.poseMat.translate(origin); poseMat.poseMat.rotate(quatEigen); // T_{cam}_{bb}
    return poseMat;
}



/**
 *
 * @param xyz query point w.r.t camera (zed_left_...)
 * @param dimensions (w,h,l) defined (x,y,z)
 * @param T_bc transform from box center to camera
 * @return
 */
bool zed_client::isPointWithinBB(double *xyz, double *dimensions, Pose T_bc,bool entireZ = false, float padding = 0) {

    Eigen::Vector4f p_cam(xyz[0],xyz[1],xyz[2],1.0);
    Eigen::Vector4f p_box = T_bc.poseMat.matrix() * p_cam; // point w.r.t box frame

    float x = p_box(0), y = p_box(1), z = p_box(2);
    float xmax = dimensions[0]/2.0f;
    float ymax = dimensions[1]/2.0f;
    float zmax = dimensions[2]/2.0f;
    if (not entireZ) {
        if (x < (xmax + padding) and x > ( -xmax - padding) and y < (ymax + padding) and y > (-ymax-padding)
            and z < (zmax+padding) and (z > -zmax -padding)) {
            return true;
        } else
            return false;
    }else{
        if (x < (xmax + padding) and x > ( -xmax - padding) and z < (zmax + padding) and z > (-zmax-padding)  ) {
            return true;
        } else
            return false;
    }
}
