//
// Created by jbs on 21. 5. 28..
//

#ifndef ZED_CLIENT_UTILS_H
#define ZED_CLIENT_UTILS_H


#include <ros/ros.h>
#include <vector>
#include <string>
#include <zed_interfaces/ObjectsStamped.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl_ros/point_cloud.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>
#include <numeric>
#include <algorithm>    // std::sort, std::stable_sort

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>

#include <compressed_depth_image_transport/codec.h>
#include <compressed_depth_image_transport/compression_common.h>
#include <compressed_image_transport/compression_common.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>


#include <Eigen/Core>
#include <Eigen/Eigenvalues>


#include <chrono>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <deque>

#ifdef OPENCV_VERSION_3_ZED
    #define CV_FONT_TIMESTAMP CV_FONT_HERSHEY_COMPLEX
    #define CV_RECT_TYPE CV_FILLED
#endif

#ifdef OPENCV_VERSION_4_ZED
    #define CV_FONT_TIMESTAMP cv::FONT_HERSHEY_COMPLEX
    #define CV_RECT_TYPE cv::FILLED
#endif


namespace enc = sensor_msgs::image_encodings;
using namespace std::chrono;
using namespace std;

namespace zed_client {

    sensor_msgs::ImagePtr imageToROSmsg(const cv::Mat& img, const std::string encodingType, std::string frameId, ros::Time t);
    bool pngDecompressDepth(const sensor_msgs::CompressedImageConstPtr& depthPtr, cv::Mat& decompressed);
    bool jpegDecompressRgb(const sensor_msgs::CompressedImageConstPtr& rgbPtr, cv::Mat& decompressed);
    void insertSkeletonPoint(const zed_interfaces::Object & object , cv::Mat& mat, Eigen::Vector3i& color, int patchSize = 20 );
    tf::Vector3 vec2Tf(Eigen::Vector3f vec) ;

    Eigen::Quaternionf averageQuaternions(const vector<Eigen::Quaternionf>& quatVec) ;



    class Timer {
        steady_clock::time_point t0;
        double measuredTime;
    public:
        Timer() { t0 = steady_clock::now(); }

        double stop(bool isMillisecond = true) {
            if (isMillisecond)
                measuredTime = duration_cast<milliseconds>(steady_clock::now() - t0).count();
            else
                measuredTime = duration_cast<microseconds>(steady_clock::now() - t0).count();
            return measuredTime;
        }
    };


    // Point
    struct Point {
        float x;
        float y;
        float z;
        Point(){x = 0; y = 0; z = 0;};
        Point(float x, float y, float z):x(x),y(y),z(z) {};
        Point(const Eigen::Vector3f & pnt):x(pnt(0)),y(pnt(1)),z(pnt(2)) {};
        Point(const geometry_msgs::Point& pnt):x(pnt.x),y(pnt.y),z(pnt.z) {};
        geometry_msgs::Point toGeometry() {
            geometry_msgs::Point pnt;
            pnt.x = x;
            pnt.y = y;
            pnt.z = z;
            return pnt;
        }
        Eigen::Vector3f toEigen() const {
            return Eigen::Vector3f(x,y,z);
        }
        Eigen::Vector3d toEigend() const {

            return Eigen::Vector3d(x,y,z);
        }
        Point operator+(const Point& pnt) const{
            return Point(pnt.x+x,pnt.y+y,pnt.z+z);
        }

        Point operator-(const Point& pnt) const {
            return Point(-pnt.x+x,-pnt.y+y,-pnt.z+z);
        }

        Point operator*(float s) const {
            return Point(s * x, s * y, s * z);
        }

        Point operator/(float s)  const{
            return *this * (1.0/s);
        }

        float distTo(Point p) const {
            return sqrt(pow(p.x-x,2) + pow(p.y-y,2)  + pow(p.z-z,2) ) ;
        }
        float dot(Point p) const{
            return x*p.x + y*p.y + z*p.z;
        }
        float norm() const {return sqrt(pow(x,2) + pow(y,2)  + pow(z,2) );}
        void normalize () { float mag = norm(); x/= mag; y /= mag;  z/= mag; }

        /**
         * |a/|a| - b/|b||
         * @param pnt
         * @return
         */
        float sphereDist(Point pnt) const {
            pnt.normalize();
            Point thisPnt = *this; thisPnt.normalize();
            return pnt.distTo(thisPnt);
        }
        pcl::PointXYZ toPCL() {
            pcl::PointXYZ xyz;
            xyz.x = x;
            xyz.y = y;
            xyz.z = z;
            return xyz;
        }

        /**
         * Get tf rotation to head vantage point from this point
         * @param vantange
         * @return tf rotation matrix
         */
        tf::Matrix3x3 alignX(const Point& vantange){
            tf::Matrix3x3 rot;
            tf::Vector3 e1(vantange.x  - x,vantange.y-y,vantange.z -z); e1.normalize();
            tf::Vector3 e2;
            e2.setX (e1.y()/sqrt( pow(e1.x(),2) + pow(e1.y(),2) ));
            e2.setY (-e1.x()/sqrt( pow(e1.x(),2) + pow(e1.y(),2) ));
            e2.setZ(0);

            tf::Vector3 e3 = e1.cross(e2);
            if (e3.z() < 0 ){
                e2 = -e2;
                e3 = e1.cross(e2);
            }

            rot.setValue(e1.x(),e2.x(),e3.x(),e1.y(),e2.y(),e3.y(),e1.z(),e2.z(),e3.z());
            return rot;
        }

    };

    Point operator*(float scalar,const Point& pnt );

    // Pose
    struct Pose{
        Eigen::Transform<float,3,Eigen::Affine> poseMat;

        void inverse() {poseMat = poseMat.inverse();}
        Pose() {poseMat.setIdentity();}
        void setTranslation(float x,float y,float z) {poseMat.translate(Eigen::Vector3f(x,y,z));};
        void setTranslation(Point pnt) {poseMat.translate(pnt.toEigen());};
        void setRotation(Eigen::Quaternionf quat) {poseMat.rotate(quat);};

        /**
         * Initialize pose at location x-axis heading to target
         * @param location pose origin
         * @param target observation target
         */
        Pose(Point location, Point target){
            poseMat.setIdentity();
            poseMat.translate(location.toEigen());
            tf::Matrix3x3 rot = location.alignX(target); tf::Quaternion quat;
            rot.getRotation(quat); Eigen::Quaternionf quatEigen(quat.w(),quat.x(),quat.y(),quat.z());
            poseMat.rotate(quatEigen);
        };
        Pose (const geometry_msgs::PoseStamped poseStamped){

            poseMat.setIdentity();
            Eigen::Vector3f loc(poseStamped.pose.position.x,
                                poseStamped.pose.position.y,
                                poseStamped.pose.position.z);

            poseMat.translate(loc);
            Eigen::Quaternionf quaternionf;
            quaternionf.setIdentity();

            quaternionf.w() = poseStamped.pose.orientation.w;
            quaternionf.x() = poseStamped.pose.orientation.x;
            quaternionf.y() = poseStamped.pose.orientation.y;
            quaternionf.z() = poseStamped.pose.orientation.z;

            poseMat.rotate(quaternionf);
        }

        Pose (const tf::StampedTransform& tfStamped){
            poseMat.setIdentity();
            Eigen::Vector3f loc(tfStamped.getOrigin().x(),tfStamped.getOrigin().y(),tfStamped.getOrigin().z());
            poseMat.translate(loc);
            Eigen::Quaternionf quaternionf;
            quaternionf.setIdentity();

            quaternionf.w() = tfStamped.getRotation().w();
            quaternionf.x() = tfStamped.getRotation().x();
            quaternionf.y() = tfStamped.getRotation().y();
            quaternionf.z() = tfStamped.getRotation().z();

            poseMat.rotate(quaternionf);
        }


        tf::StampedTransform toTf(string worldFrameName, string frameName,ros::Time time) const {
            tf::StampedTransform stampedTransform;
            stampedTransform.frame_id_ = worldFrameName;
            stampedTransform.child_frame_id_ = frameName;
            stampedTransform.stamp_ = time;
            stampedTransform.setIdentity();

            tf::Vector3 vec(poseMat.translation().x(),poseMat.translation().y(),poseMat.translation().z());
            stampedTransform.setOrigin(vec);

            Eigen::Quaternionf quatt(poseMat.rotation());
            tf::Quaternion quat(quatt.x(),quatt.y(),quatt.z(),quatt.w());
            stampedTransform.setRotation(quat);
            return stampedTransform;
        }


        Point getTranslation() const {
            Point pnt;
            pnt.x = poseMat.translation().x();
            pnt.y = poseMat.translation().y();
            pnt.z = poseMat.translation().z();
            return pnt;
        }


        /**
         * (x,y,z,w)
         * @return
         */
        Eigen::Quaternionf getQuaternion(){
            return Eigen::Quaternionf(poseMat.rotation());

        }

        void rotate(Eigen::Vector3f axis,float angle){
            poseMat.rotate(Eigen::AngleAxisf(angle,axis));
        }

        void print() const {
            std::cout << poseMat.matrix() <<std::endl;
        }

        geometry_msgs::Pose toGeoPose() const {
            geometry_msgs::Pose pose;

            tf::Vector3 vec(poseMat.translation().x(),poseMat.translation().y(),poseMat.translation().z());
            Eigen::Quaternionf quatt(poseMat.rotation());

            pose.position.x = vec.x();
            pose.position.y = vec.y();
            pose.position.z = vec.z();

            pose.orientation.x = quatt.x();
            pose.orientation.y = quatt.y();
            pose.orientation.z = quatt.z();
            pose.orientation.w = quatt.w();

            return pose;
        }

        void applyTransform (const Pose& pose){
            poseMat = pose.poseMat * poseMat;
        }

    };

    Point operator*(float scalar,const Point& pnt );

    Eigen::Vector3f corner2Vec(zed_interfaces::Keypoint3D corner);
    // check whether point is within box
    bool isPointWithinBB(double xyz_c[3], double dimensions[3], Pose T_bc, bool entireZ, float padding);
    // extract T_cb
    Pose extractBbPoseFromSensor(const zed_interfaces::Object &object);


}
#endif //ZED_CLIENT_UTILS_H
