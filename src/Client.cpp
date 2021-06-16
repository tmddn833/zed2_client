//
// Created by jbs on 21. 6. 3..
//


#include <zed2_client/Client.h>

using namespace zed_client;
using namespace std;

cv::Vec3b DetectedObject::getHsvColor() const {
    cv::Mat bgr = cv::Mat(1, 1, CV_8UC3, bgrColor);
    cv::Mat hsv;
    cv::cvtColor(bgr,hsv,cv::COLOR_BGR2HSV);
    return hsv.at<cv::Vec3b>(0, 0);
}

bool DetectedObject::isPointWithinBB(const Point &pnt_w, bool entireZ, float padding) const {
    float xmax = zedRawData.dimensions_3d[0]/2.0f;
    float ymax = zedRawData.dimensions_3d[1]/2.0f;
    float zmax = zedRawData.dimensions_3d[2]/2.0f;

    Point pnt_b =  bbPose_w.poseMat.inverse() * pnt_w.toEigen();
    float x = pnt_b.x;
    float y = pnt_b.y;
    float z = pnt_b.z;

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



cv::Vec3b TrackedObject::getHsvColor() const {
    cv::Mat bgr = cv::Mat(1, 1, CV_8UC3,filteredColorQueue.back());
    cv::Mat hsv;
    cv::cvtColor(bgr,hsv,cv::COLOR_BGR2HSV);
    return hsv.at<cv::Vec3b>(0, 0);
}


TrackedObject::TrackedObject(DetectedObject obj, int targetIdx,
                             float heightOffSetForFilterPose, float smoothing,
                             int queueSize,int minPntForPoseInference) {
    clientLastUpdateStamp = ros::Time::now();
    beingTrackedAs = targetIdx;
    bbPose_w = obj.bbPose_w;
    bgrColor = obj.bgrColor;
    zedRawData = obj.zedRawData;

    p_heightOffset = heightOffSetForFilterPose;
    p_smoothingWeight = smoothing;
    p_queueSize = queueSize;
    p_minPntForPoseInference = minPntForPoseInference;

    Pose pose = bbPose_w;
    pose.poseMat.translate(Eigen::Vector3f(0, -p_heightOffset, 0));
    filteredPointQueue.push_back(pose.getTranslation());

    filteredColorQueue.push_back(bgrColor);
}


void TrackedObject::update(const DetectedObject & newObj) {
    float dt = (newObj.zedStamp - zedStamp).toSec();
    zedStamp = newObj.zedStamp;
    clientLastUpdateStamp = ros::Time::now();
    zedRawData = newObj.zedRawData;
    bbPose_w= newObj.bbPose_w;
    bgrColor= newObj.bgrColor;

    Pose pose = newObj.bbPose_w;
    pose.poseMat.translate(Eigen::Vector3f(0, -p_heightOffset, 0));
    Point updatePnt;
    if( not filteredPointQueue.empty() ) {
        updatePnt = (1 - p_smoothingWeight) * pose.getTranslation() + p_smoothingWeight * filteredPointQueue.back();
    }
    else
        updatePnt = pose.getTranslation();

    if (dt > 0) {
        Point curVel = (updatePnt - filteredPointQueue.back()) * (1.0f / dt);
        Point updateVel = (1 - p_smoothingWeight) * curVel + p_smoothingWeight * velocityFiltered_w;
        velocityFiltered_w = updateVel;
        velocityComputed = true;
    }
    filteredPointQueue.push_back(updatePnt);
    if (filteredPointQueue.size() > p_queueSize)
        filteredPointQueue.pop_front();

    cv::Vec3f color = newObj.bgrColor;
    cv::Vec3f updateColorf;
    if (not filteredColorQueue.empty())
        updateColorf = (1 - p_smoothingWeight) * color + p_smoothingWeight * cv::Vec3f(filteredColorQueue.back()) ;
    else
        updateColorf = color;
    filteredColorQueue.push_back(cv::Vec3i(updateColorf));
    if (filteredColorQueue.size() > p_queueSize)
        filteredColorQueue.pop_front();

}

pcl::PointCloud<pcl::PointXYZRGB> TrackedObject::getObservationQueuePCL(string worldFrameId) {
    int currentQueueSize = filteredPointQueue.size();
    pcl::PointCloud<pcl::PointXYZRGB> points;
    points.header.frame_id = worldFrameId;
    points.header.stamp = pcl_conversions::toPCL(zedStamp);

    for (int n = 0 ; n < currentQueueSize ; n++) {
        pcl::PointXYZRGB pnt;
        pnt.x = filteredPointQueue[n].x;
        pnt.y = filteredPointQueue[n].y;
        pnt.z = filteredPointQueue[n].z;
        pnt.r = filteredColorQueue[n](2);
        pnt.g = filteredColorQueue[n](1);
        pnt.b = filteredColorQueue[n](0);
        points.push_back(pnt);
    }
    return points;
}

bool TrackedObject::getFilteredPoseFromQueue(Pose &pose) const {
    if (filteredPointQueue.size() < p_minPntForPoseInference) {
        ROS_WARN("Too few points for x-direction inference < %d. "
                 "target filtered tf will not be broadcast",
                 p_minPntForPoseInference);
        return false;
    }

    int nUseKF = int(filteredPointQueue.size()/ 3.0f) ;
    int nUse = 0;
    vector<Eigen::Quaternionf> quatStack;
    for (auto it =filteredPointQueue.rbegin() ; nUse < nUseKF-1 ; it++  ){
        Point p2 = *it;
        Point p1 = *(it+1);
        Pose dir(p1,p2);
        quatStack.push_back(dir.getQuaternion());
        nUse++;
    }
    Eigen::Quaternionf quatAvg = averageQuaternions(quatStack);
    pose.setTranslation(filteredPointQueue.back());
    pose.setRotation(quatAvg);
    return true;
}

// todo prediction result
bool TrackedObject::isPointWithinBB(const Point &pnt_w, bool entireZ, float padding) const {
    if (velocityComputed && (ros::Time::now()-clientLastUpdateStamp).toSec() > 0.1){
        // do not care much about exact shape of 2d section shape
        float boxWidth = (zedRawData.dimensions_3d[0] + zedRawData.dimensions_3d[2])/2.0 + 2*padding;
        float boxHeight = (entireZ) ? numeric_limits<float>::max() :  zedRawData.dimensions_3d[1] + 2*padding;
        Point trackedCenter = getLinearPredictionPoint();
        return
            abs(pnt_w.x - trackedCenter.x) < boxWidth/2.0 and
            abs(pnt_w.y - trackedCenter.y) < boxWidth/2.0 and
            abs(pnt_w.z - trackedCenter.z) < boxHeight/2.0;
    }else{
        return DetectedObject::isPointWithinBB(pnt_w,entireZ,padding);
    }

}

geometry_msgs::PointStamped TrackedObject::getLinearPredictionPoint(string worldFrame) {
    geometry_msgs::PointStamped point;
    if (velocityComputed) {
        ros::Time requestTime = ros::Time::now();
        point.header.frame_id = worldFrame;
        Point prediction =
                filteredPointQueue.back() + velocityFiltered_w * (requestTime - clientLastUpdateStamp).toSec();
        point.point = prediction.toGeometry();
    }
    return point;
}

Point TrackedObject::getLinearPredictionPoint() const {
    Point vel_w;
    if (velocityComputed){
        ros::Time requestTime = ros::Time::now();
        vel_w = filteredPointQueue.back() + velocityFiltered_w * (requestTime - clientLastUpdateStamp).toSec();
    }
    return vel_w;
}

Client::Client() :nh("~"), it (nh) {

    // paramter parsing
    nh.param<string>("world_frame_id",param.worldFrame,"map");
    nh.param<bool>("mask_object",param.filterObject,true);
    nh.param<bool>("additional_pcl",param.additionalPcl,true);

    nh.param("target_tracking/n_target",param.nTarget,2);
    nh.param("target_tracking/height_offset_from_box_center",param.heightOffsetFromBbCenter,0.5f);
    nh.param("target_tracking/callback_interval",param.targetIdInterval,0.08f);
    nh.param("target_tracking/smoothing_weight",param.smoothingWeight,0.8f);
    nh.param("target_tracking/queue",param.queueSize,50);
    nh.param("target_tracking/min_pts_for_pose_inference",param.minimumPointsForPoseInference,10);

    nh.param("target_detection/matching_weight/color",param.MW_color,1.0f);
    nh.param("target_detection/matching_weight/location",param.MW_location,1.0f);
    nh.param("target_detection/matching_weight/velocity",param.MW_velocity,1.0f);

    nh.param("target_detection/assumption/dist_accept",param.M_distAcceptThreshold,0.7f);
    nh.param("target_detection/assumption/color_accept",param.M_colorAcceptThreshold,100.0f);
    nh.param("target_detection/assumption/color_reject",param.M_colorRejectThreshold,255.0f);
    nh.param("target_detection/assumption/dist_reject",param.M_distRejectionThreshold,1.5f);

    nh.param("pointcloud/mask_padding_x",param.maskThicknessX,10);
    nh.param("pointcloud/mask_padding_y",param.maskThicknessY,20);
    nh.param("pointcloud/bounding_box_padding",param.bbPadding,0.3f);
    nh.param("pointcloud/publish_after_targets_fixed",param.publishPclAfterTargetLocked,true);
    nh.param("pointcloud/stride",param.pclStride,5);
    nh.param("pointcloud/filter_speckle",param.filterSpeckle,true);
    nh.param("pointcloud/speckle_search_rad",param.speckleSearchRadius,0.5f);
    nh.param("pointcloud/speckle_search_neighbors",param.speckleNeighbors,5);



    subDepthComp = new message_filters::Subscriber<sensor_msgs::CompressedImage>(nh,"/zed2/zed_node/depth/depth_registered/compressedDepth",1);
    subRgbComp = new message_filters::Subscriber<sensor_msgs::CompressedImage>(nh,"/zed2/zed_node/rgb/image_rect_color/compressed",1);
    subZedOd = new message_filters::Subscriber<zed_interfaces::ObjectsStamped>(nh,"/zed2/zed_node/obj_det/objects",1);
    subCamInfo = new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh,"/zed2/zed_node/rgb/camera_info",1);
    subAdditionalPcl = new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh,"/d435/depth/color/points",5);

    if (param.filterObject) {
        if (param.additionalPcl){

            subSyncPcl = new message_filters::Synchronizer<CompressedImageMaskBbPclSync>(CompressedImageMaskBbPclSync(10),
                                                                                         *this->subRgbComp, *this->subDepthComp,
                                                                                         *this->subCamInfo, *this->subZedOd,
                                                                                         *this->subAdditionalPcl);

            subSyncPcl->registerCallback(boost::bind(&Client::zedPclSyncCallback, this, _1, _2, _3, _4, _5));

        }else{
            subSync = new message_filters::Synchronizer<CompressedImageMaskBbSync>(CompressedImageMaskBbSync(10),
                                                                                         *this->subRgbComp, *this->subDepthComp,
                                                                                         *this->subCamInfo, *this->subZedOd
                                                                                         );

            subSync->registerCallback(boost::bind(&Client::zedSyncCallback, this, _1, _2, _3, _4));
        }
    }else{

        subSyncSimple = new message_filters::Synchronizer<CompressedImageSync>(CompressedImageSync(10),
                                                                               *this->subRgbComp, *this->subDepthComp,
                                                                               *this->subCamInfo);
        subSyncSimple->registerCallback(boost::bind(&Client::zedSyncCallback, this, _1, _2, _3));

    }

    pubRgbMaskImg =  it.advertise("image_rect_color_masked",1);
    pubDepthMaskImg =  it.advertise("depth_masked",1);
    pubPointsMasked = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("points_masked",1);
    pubPointRemoved = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("points_removed",1);

    for(int n = 0 ; n < param.nTarget ; n++) {
        pubObservationFiltered.push_back(
                nh.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("observation_filtered/target_" + to_string(n), 1));
        pubTargetLinearPrediction.push_back(
                nh.advertise<geometry_msgs::PointStamped>("simple_prediction/target_" + to_string(n),1));
    }
    tfListenerPtr = new tf::TransformListener;
    tfBroadcasterPtr = new tf::TransformBroadcaster;

    nhGlobal.setCallbackQueue(&observationCallbackQueue);
    timerCaller  =nhGlobal.createTimer(ros::Duration(param.targetIdInterval),
                                       &Client::targetIdCallback, this);
    targetIdSpinnerPtr = new ros::AsyncSpinner(1, &observationCallbackQueue); // this spinner is dedicated to timer callback only. intended not to invoke updating mesh
    targetIdSpinnerPtr->start();
}

float Client::matchingCost(const TrackedObject &priorObj, const DetectedObject &newObj) const {

    float distDiff = (priorObj.bbPose_w.getTranslation() - newObj.bbPose_w.getTranslation()).norm();
    float velDiff = sqrt(pow (priorObj.zedRawData.velocity[0] - newObj.zedRawData.velocity[0] , 2) +
            pow (priorObj.zedRawData.velocity[1] - newObj.zedRawData.velocity[1] , 2) +
            pow (priorObj.zedRawData.velocity[2] - newObj.zedRawData.velocity[2] , 2));

    cv::Vec3b colorPrior = priorObj.getHsvColor();
    cv::Vec3b colorNew = newObj.getHsvColor();
    float colorDiff = sqrt(pow(colorPrior(0) - colorNew(0),2) +
            pow(colorPrior(1) - colorNew(1),2)); // using H and S only

    // accept assumption 1
    if ( (priorObj.zedRawData.label_id == newObj.zedRawData.label_id) ) {
        ROS_DEBUG("Accept: only one received obj and same label %d. accepting!",  newObj.zedRawData.label_id);
        return 0.0;
    }

    // accept assumption 2
    if (colorDiff < param.M_colorAcceptThreshold and distDiff < param.M_distAcceptThreshold) {
        ROS_DEBUG("Accept: comparing label %d and %d. Color diff = %f and dist diff = %f. accepting!",
                 priorObj.zedRawData.label_id, newObj.zedRawData.label_id,
                 colorDiff,distDiff);
        return param.MW_velocity * velDiff;
    }

    // reject assumption 1
    if (colorDiff > param.M_colorRejectThreshold) {
        ROS_DEBUG("Reject: comparing label %d with prior hsv (%d,%d,%d) of %d th target. Color diff = %f too big.",
                 newObj.zedRawData.label_id,
                 colorPrior(0), colorPrior(1), colorPrior(2),
                 priorObj.beingTrackedAs, colorDiff);
        return MATCHING_COST_INF;
    }

    // rejection assumption 2
    if (distDiff > param.M_distRejectionThreshold){
        ROS_DEBUG("Reject: target  [%d] and tested obj [%d]= dist diff = %f too big.",
                 priorObj.beingTrackedAs, newObj.zedRawData.label_id,distDiff);
        return MATCHING_COST_INF;
    }

    return param.MW_location * distDiff + param.MW_velocity * velDiff + param.MW_color * colorDiff;
}


void Client::zedSyncCallback(const sensor_msgs::CompressedImageConstPtr & rgbCompPtr, const sensor_msgs::CompressedImageConstPtr & depthCompPtr,
                             const sensor_msgs::CameraInfoConstPtr & camInfoPtr ) {

    // time recording
    ros::Time curSensorTime = depthCompPtr->header.stamp;
    state.clientLastCallTime = ros::Time::now();

    double fps = 1.0 / (curSensorTime - state.zedLastCallTime).toSec();
    ROS_DEBUG("sync callback fps = %f " ,  fps);
    state.zedLastCallTime = curSensorTime;

    // find tf from map to cam
    tf::StampedTransform transform;
    try {
        // time 0 in lookup was intended
        tfListenerPtr->lookupTransform(param.worldFrame, depthCompPtr->header.frame_id,
                                       curSensorTime, transform);
        state.T_wc = Pose(transform);
        state.T_cw = state.T_wc; state.T_cw.inverse();

    }catch (tf::TransformException& ex) {
        ROS_ERROR_STREAM(ex.what());
        ROS_ERROR("[ZedClient] no transform between map and object header. Cannot process further.");
        return;
    }

    // decompress and parse cam model
    cv::Mat depthImg,rgbImg;
    bool isCompressionSuccess = pngDecompressDepth(depthCompPtr, depthImg) and
                                jpegDecompressRgb(rgbCompPtr,rgbImg);
    if (not isCompressionSuccess) {
        ROS_ERROR("image decompression error");
        return;
    }

    Timer pclTimer;
    image_geometry::PinholeCameraModel model_;
    model_.fromCameraInfo(*camInfoPtr);
    double camera_cx = model_.cx();
    double camera_cy = model_.cy();
    double camera_fx = model_.fx();
    double camera_fy = model_.fy();
    double camera_factor = 1;

    state.pclObjectsRemoved.clear();
    state.pclObjectsRemoved.header.frame_id = depthCompPtr->header.frame_id;
    state.pclObjectsRemoved.header.stamp = pcl_conversions::toPCL(curSensorTime);

    state.pclFurtherRemoved.clear();
    state.pclFurtherRemoved.header = state.pclObjectsRemoved.header;

    for (int r = 0; r < depthImg.rows; r += param.pclStride) {
        for (int c = 0; c < depthImg.cols; c += param.pclStride) {
            float d = depthImg.ptr<float>(r)[c];
            auto ptr_r = rgbImg.ptr<uchar>(r);
            if (d != d) // nan
                continue;
            pcl::PointXYZRGB p;
            p.z = double(d) / camera_factor;
            p.x = (c - camera_cx) * p.z / camera_fx;
            p.y = (r - camera_cy) * p.z / camera_fy;
            p.r = ptr_r[3 * c + 2];
            p.g = ptr_r[3 * c + 1];
            p.b = ptr_r[3 * c + 0];
            state.pclObjectsRemoved.points.push_back(p);
        }
    }

    // speckle removal
    if (param.filterSpeckle) {
        int originalPoints = state.pclObjectsRemoved.points.size();
        pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> outrem;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>(state.pclObjectsRemoved));
        outrem.setInputCloud(cloud);
        outrem.setRadiusSearch(param.speckleSearchRadius);
        outrem.setMinNeighborsInRadius(param.speckleNeighbors);
        vector<int> survivalIndex;
        outrem.filter(survivalIndex);
        state.pclObjectsRemoved.points.clear();
        int j = 0;
        for (int i = 0; i < originalPoints; i++) {
            if (i == survivalIndex[j]) {
                state.pclObjectsRemoved.points.push_back(cloud->points[i]);
                j++;
            } else
                state.pclFurtherRemoved.push_back(cloud->points[i]);
        }
        ROS_DEBUG("speckle removed: %d", originalPoints - state.pclObjectsRemoved.size());
    }

    state.pclObjectsRemoved.header.frame_id = depthCompPtr->header.frame_id;
    state.pclObjectsRemoved.header.stamp = pcl_conversions::toPCL(curSensorTime);
    double pclElapse = pclTimer.stop();
    ROS_DEBUG("pcl generation took %f ms", pclElapse);

    pubRgbMaskImg.publish(imageToROSmsg(rgbImg, enc::BGR8, rgbCompPtr->header.frame_id, curSensorTime));
    pubDepthMaskImg.publish(imageToROSmsg(depthImg, enc::TYPE_32FC1, rgbCompPtr->header.frame_id, curSensorTime));
    pubPointsMasked.publish(state.pclObjectsRemoved);
    pubPointRemoved.publish(state.pclFurtherRemoved);


}



void Client::syncSubRoutine(const sensor_msgs::CompressedImageConstPtr & rgbCompPtr, const sensor_msgs::CompressedImageConstPtr & depthCompPtr,
                                const sensor_msgs::CameraInfoConstPtr & camInfoPtr, const zed_interfaces::ObjectsStampedConstPtr & objPtr ,
                                const sensor_msgs::PointCloud2ConstPtr & pclPtr) {
    // time recording
    ros::Time curSensorTime = depthCompPtr->header.stamp;
    double fps = 1.0 / (curSensorTime - state.zedLastCallTime).toSec();
    ROS_DEBUG("sync callback fps = %f " ,  fps);
    state.zedLastCallTime = curSensorTime;
    state.clientLastCallTime = ros::Time::now();

    // find tf from map to cam
    tf::StampedTransform transform;

    // find tf from map to cam
    try {
        // time 0 in lookup was intended
        tfListenerPtr->lookupTransform(param.worldFrame, depthCompPtr->header.frame_id,
                                       curSensorTime, transform);
        state.T_wc = Pose(transform);
        state.T_cw = state.T_wc; state.T_cw.inverse();

    }catch (tf::TransformException& ex) {
        ROS_ERROR_STREAM(ex.what());
        ROS_ERROR("[ZedClient] no transform between map and object header. Cannot process further.");
        return;
    }

    if (pclPtr != nullptr) {
        // find tf from map to zed cam to d435
        try {
            // time 0 in lookup was intended
            tfListenerPtr->lookupTransform(depthCompPtr->header.frame_id,
                                           pclPtr->header.frame_id,
                                           curSensorTime, transform);
            state.T_cd = Pose(transform);

        } catch (tf::TransformException &ex) {
            ROS_ERROR_STREAM(ex.what());
            ROS_ERROR("[ZedClient] no transform between zed and d435. Cannot process further.");
            return;
        }
    }


    // find tf from map to object incoming frame
    try {
        // time 0 in lookup was intended
        tfListenerPtr->lookupTransform(param.worldFrame, objPtr->header.frame_id,
                                       curSensorTime, transform);
        state.T_wo = Pose(transform);
    }catch (tf::TransformException& ex) {
        ROS_ERROR_STREAM(ex.what());
        ROS_ERROR("[ZedClient] no transform between map and object header. Cannot process further.");
        return;
    }

    // decompress and parse cam model
    cv::Mat depthImg,rgbImg;
    bool isCompressionSuccess = pngDecompressDepth(depthCompPtr, depthImg) and
                                jpegDecompressRgb(rgbCompPtr,rgbImg);
    if (not isCompressionSuccess) {
        ROS_ERROR("image decompression error");
        return;
    }

    // masking and color id
    int rGlobalMin = 0;
    int cGlobalMin = 0;
    int rGlobalMax = rgbImg.rows - 1;
    int cGlobalMax = rgbImg.cols - 1;
    const float NaN = std::numeric_limits<float>::quiet_NaN() ;
    vector<DetectedObject> newObjects;
    for (const auto &obj :objPtr->objects) {

        ROS_DEBUG("number of detected obj: %d ", objPtr->objects.size());
        Eigen::Vector3i objColor;
        insertSkeletonPoint(obj,rgbImg,objColor);

        int rMin = max(int(obj.bounding_box_2d.corners[0].kp[1]) - param.maskThicknessY, rGlobalMin);
        int cMin = max(int(obj.bounding_box_2d.corners[0].kp[0]) - param.maskThicknessX, cGlobalMin);
        int rMax = min(int(obj.bounding_box_2d.corners[2].kp[1]) + param.maskThicknessY, rGlobalMax);
        int cMax = min(int(obj.bounding_box_2d.corners[2].kp[0]) + param.maskThicknessX, cGlobalMax);
        float alpha = 0.5;

        // masking for depth (set nan) and rgb (coloring with average color)
        // NOTE: even the targets are lost for a short moment, we keep removing the bounding box from the last bounding box
        for (int r = rMin; r < rMax; r++) {
            auto ptr_r =rgbImg.ptr<uchar>(r);
            auto ptr_depth =depthImg.ptr<float>(r);
            for (int c = cMin; c < cMax; c++) {
                ptr_r[3 * c + 0] = (int) (alpha * ptr_r[3 * c + 0] + (1 - alpha) * objColor.z());
                ptr_r[3 * c + 1] = (int) (alpha * ptr_r[3 * c + 1] + (1 - alpha) * objColor.y());
                ptr_r[3 * c + 2] = (int) (alpha * ptr_r[3 * c + 2] + (1 - alpha) * objColor.x());
                ptr_depth[c] = NaN;
            }
        }

        if (obj.tracking_state == 2) { // if state is SEARCHING, this is in occlusion or tracking lost
            ROS_ERROR("object %d occluded or interrupted..", obj.label_id);
            continue;
        }

        Pose T_ob; // pose from to box (box = optical frame, z-forwarding)
        DetectedObject newObj;
        newObj.zedStamp = depthCompPtr->header.stamp;
        newObj.zedRawData = obj;
        newObj.bgrColor = cv::Vec3i(objColor.z(), objColor.y(), objColor.x());
        T_ob = extractBbPoseFromSensor(obj);
        Point v_o = Point(newObj.zedRawData.velocity[0],
                          newObj.zedRawData.velocity[1],
                          newObj.zedRawData.velocity[2]);
        Eigen::Matrix3f R_bo = T_ob.poseMat.rotation().matrix().inverse();
        newObj.bbPose_w = T_ob;
        newObj.bbPose_w.applyTransform(state.T_wo);
//        tfBroadcasterPtr->sendTransform(newObj.bbPose_w.toTf(param.worldFrame,"object_" + to_string(obj.label_id), curSensorTime));
        tfBroadcasterPtr->sendTransform(newObj.bbPose_w.toTf(param.worldFrame,"object_" + to_string(obj.label_id),state.clientLastCallTime));
        newObjects.push_back(newObj);
    }

    // update object info
    if(mutex_.try_lock()) {
        state.receivedObjects = newObjects;
        mutex_.unlock();
    }else{
        ROS_DEBUG("object cannot be updated due to lock from timer callback thread");
    }

    // create pcl
    bool createPcl = false;
    if(param.publishPclAfterTargetLocked ){
        createPcl = state.isAllTargetsTracked;
        if(not createPcl)
            ROS_WARN("not making pcl from decompressed depth until targets are locked. (publishPclAfterTargetLocked = true was set)");
    }else
        createPcl = true;
    if(createPcl) {
        Timer pclTimer;
        image_geometry::PinholeCameraModel model_;
        model_.fromCameraInfo(*camInfoPtr);
        double camera_cx = model_.cx();
        double camera_cy = model_.cy();
        double camera_fx = model_.fx();
        double camera_fy = model_.fy();
        double camera_factor = 1;
        state.pclObjectsRemoved.clear();
        state.pclObjectsRemoved.header.frame_id = depthCompPtr->header.frame_id;
        state.pclObjectsRemoved.header.stamp = pcl_conversions::toPCL(curSensorTime);

        state.pclFurtherRemoved.clear();
        state.pclFurtherRemoved.header = state.pclObjectsRemoved.header;

        for (int r = 0; r < depthImg.rows; r += param.pclStride) {
            for (int c = 0; c < depthImg.cols; c += param.pclStride) {
                float d = depthImg.ptr<float>(r)[c];
                auto ptr_r = rgbImg.ptr<uchar>(r);
                if (d != d) // nan
                    continue;
                pcl::PointXYZRGB p;
                p.z = double(d) / camera_factor;
                p.x = (c - camera_cx) * p.z / camera_fx;
                p.y = (r - camera_cy) * p.z / camera_fy;
                p.r = ptr_r[3 * c + 2];
                p.g = ptr_r[3 * c + 1];
                p.b = ptr_r[3 * c + 0];

                // double check for masking with 3d if targets are locked
                if (state.isAllTargetsTracked){
                    bool isPointInTarget = false;
                    for (const auto & target : state.targetObjects){
                        Point p_w = state.T_wc.poseMat * Point(p.x,p.y,p.z).toEigen();

                        if (target.isPointWithinBB(p_w,true,param.bbPadding)) {
                            isPointInTarget = true;
                            break;
                        }
                    }
                    if (isPointInTarget) {
                        state.pclFurtherRemoved.push_back(p);
                        continue;
                    }
                }
                state.pclObjectsRemoved.points.push_back(p);
            }
        }
        if (pclPtr != nullptr) {
            // additional pointcloud (assume no objects filtering required for them)
            pcl::PointCloud<pcl::PointXYZRGB> pclCloud;
            pcl::fromROSMsg(*pclPtr, pclCloud);
            for (int n = 0; n < pclCloud.size(); n += pow(param.pclStride, 2)) {
                auto point = pclCloud.points[n];
                Point p_c = state.T_cd.poseMat * Point(point.x, point.y, point.z).toEigen();
                pcl::PointXYZRGB pclPnt;
                pclPnt.x = p_c.x;
                pclPnt.y = p_c.y;
                pclPnt.z = p_c.z;
                pclPnt.r = point.r;
                pclPnt.g = point.g;
                pclPnt.b = point.b;
                state.pclObjectsRemoved.points.push_back(pclPnt);
            }

            ROS_DEBUG("addition pcl added: %d", pclCloud.size());
        }



        // speckle removal
        if (param.filterSpeckle) {
            int originalPoints = state.pclObjectsRemoved.points.size();
            pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> outrem;
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>(state.pclObjectsRemoved));
            outrem.setInputCloud(cloud);
            outrem.setRadiusSearch(param.speckleSearchRadius);
            outrem.setMinNeighborsInRadius(param.speckleNeighbors);
            vector<int> survivalIndex;
            outrem.filter(survivalIndex);
            state.pclObjectsRemoved.points.clear();
            int j = 0;
            for (int i = 0; i < originalPoints; i++) {
                if (i == survivalIndex[j]) {
                    state.pclObjectsRemoved.points.push_back(cloud->points[i]);
                    j++;
                } else
                    state.pclFurtherRemoved.push_back(cloud->points[i]);
            }
            ROS_DEBUG("speckle removed: %d", originalPoints - state.pclObjectsRemoved.size());
        }

        state.pclObjectsRemoved.header.frame_id = depthCompPtr->header.frame_id;
        state.pclObjectsRemoved.header.stamp = pcl_conversions::toPCL(curSensorTime);
        double pclElapse = pclTimer.stop();
        ROS_DEBUG("pcl generation took %f ms", pclElapse);
    }

    // publish
    pubRgbMaskImg.publish(imageToROSmsg(rgbImg, enc::BGR8, rgbCompPtr->header.frame_id, curSensorTime));
    pubDepthMaskImg.publish(imageToROSmsg(depthImg, enc::TYPE_32FC1, rgbCompPtr->header.frame_id, curSensorTime));
    pubPointsMasked.publish(state.pclObjectsRemoved);
    pubPointRemoved.publish(state.pclFurtherRemoved);
}

void Client::zedPclSyncCallback(const sensor_msgs::CompressedImageConstPtr & compImgPtr,
                                const sensor_msgs::CompressedImageConstPtr & depthImgPtr,
                                const sensor_msgs::CameraInfoConstPtr & camPtr,
                                const zed_interfaces::ObjectsStampedConstPtr & objPtr,
                                const sensor_msgs::PointCloud2ConstPtr & pclPtr) {
    syncSubRoutine(compImgPtr,depthImgPtr,camPtr,objPtr,pclPtr);
}


void Client::zedSyncCallback(const sensor_msgs::CompressedImageConstPtr & compImgPtr,
                                const sensor_msgs::CompressedImageConstPtr & depthImgPtr,
                                const sensor_msgs::CameraInfoConstPtr & camPtr,
                                const zed_interfaces::ObjectsStampedConstPtr & objPtr
                                ) {
    syncSubRoutine(compImgPtr,depthImgPtr,camPtr,objPtr, nullptr);
}


void Client::targetIdCallback(const ros::TimerEvent &event) {

    mutex_.lock();
    auto newObjects = state.receivedObjects;
    mutex_.unlock();

    // update tracked objects
    if (not newObjects.empty()) {

        // if no targets locked, find the closest ones;
        if (not state.isAllTargetsTracked) {
            if (newObjects.size() >= param.nTarget)  {

                std::vector<float> distToCam;
                for (const auto &object: newObjects) {
                    float x = object.zedRawData.position[0];
                    float y = object.zedRawData.position[1];
                    float z = object.zedRawData.position[2];
                    float dist = sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));
                    distToCam.push_back(dist);
                }

                std::vector<size_t> idx(distToCam.size());
                iota(idx.begin(), idx.end(), 0);
                stable_sort(idx.begin(), idx.end(),
                            [& distToCam](size_t i1, size_t i2) { return distToCam[i1] < distToCam[i2]; });

                for (int n = 0; n < param.nTarget; n++)  {
                    DetectedObject newObj = newObjects[idx[n]];
                    TrackedObject trackedObj(newObj, n,
                                             param.heightOffsetFromBbCenter,
                                             param.smoothingWeight,
                                             param.queueSize,param.minimumPointsForPoseInference );

                    state.targetObjects.push_back(trackedObj);
                }
                state.isAllTargetsTracked = true;
            }
        } else { // if targets were locked, match tracked Obj ~ detected obj

            int nNewObjects = newObjects.size();
            // priority of each tracked objects. But they are just wishlist in case of dual-target case.
            std::vector<std::vector<int>> selectionPriority(param.nTarget);
            std::vector<std::vector<float>> matchingCosts(param.nTarget);

            for (int idxOld = 0; idxOld < param.nTarget; idxOld++) {
                for (int idxNew = 0; idxNew < nNewObjects; idxNew++) {
                    matchingCosts[idxOld].push_back(matchingCost(state.targetObjects[idxOld],
                                                                 newObjects[idxNew]));
                    selectionPriority[idxOld].push_back(idxNew); // before sorting
                }
                std::vector<float> matchingCostString = matchingCosts[idxOld];
                std::sort(selectionPriority[idxOld].begin(), selectionPriority[idxOld].end(),
                          [& matchingCostString](size_t i1, size_t i2) {
                              return matchingCostString[i1] < matchingCostString[i2];
                          });
            }

            // Single target case, just pick the best one as the updated target (if certain conditions are met)
            if (param.nTarget == 1) {
                if (matchingCosts[0][selectionPriority[0][0]] < MATCHING_COST_INF)
                    state.targetObjects[0].update(newObjects[selectionPriority[0][0]]);

            } else { // In dual target case, winner take priority if there is conviction

                int bestPicks[2] = {selectionPriority[0][0], selectionPriority[1][0]};
                float bestCost[2] = {matchingCosts[0][bestPicks[0]], matchingCosts[1][bestPicks[1]]};

                // This case can occur either one of the two.
                // 1) only one object was received. The target having larger matching will take that guy.
                // 2) multiple objects were received, however, competing case for the two targets.
                if (bestPicks[0] == bestPicks[1]) {

                    // 1) update only dominant object if it is make-sense matching
                    if (nNewObjects == 1) {
                        int winner = (bestCost[0] < bestCost[1]) ? 0 : 1;
                        if (bestCost[winner] < MATCHING_COST_INF)
                            state.targetObjects[winner].update(newObjects[bestPicks[winner]]);

                    // 2) in multiple object case, only give the winner the priority while other will take secondary priority
                    } else {
                        (bestCost[0] < bestCost[1]) ?
                                bestPicks[1] = selectionPriority[1][1] : bestPicks[0] = selectionPriority[0][1];
                        for (int m = 0; m < 2; m++) {
                            bestCost[m] = matchingCosts[m][bestPicks[m]];
                            if (bestCost[m] < MATCHING_COST_INF)
                                state.targetObjects[m].update(newObjects[bestPicks[m]]);
                        }
                    }

                // In this case, there were at least more than two new objects
                } else
                    for (int m = 0; m < 2; m++)
                        if (bestCost[m] < MATCHING_COST_INF)
                            state.targetObjects[m].update(newObjects[bestPicks[m]]);
            }
        }
    }

    // publish
    if (state.isAllTargetsTracked)
        for (int n = 0; n < param.nTarget ; n++){
            pubObservationFiltered[n].publish(
                    state.targetObjects[n].getObservationQueuePCL(param.worldFrame));
            pubTargetLinearPrediction[n].publish(state.targetObjects[n].getLinearPredictionPoint(param.worldFrame));
            Pose targetPose;
            if (state.targetObjects[n].getFilteredPoseFromQueue(targetPose)){
                tfBroadcasterPtr->sendTransform(targetPose.toTf(param.worldFrame,
                                                                "target_" + to_string(n) + "_filtered", state.targetObjects[n].clientLastUpdateStamp));
            }


        }
}









