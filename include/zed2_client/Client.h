//
// Created by jbs on 21. 6. 3..
//

#ifndef ZED2_CLIENT_CLIENT_H
#define ZED2_CLIENT_CLIENT_H

#include <zed2_client/Utils.h>

#define MATCHING_COST_INF 99999 // to reject non-sense matching


namespace zed_client{

    typedef message_filters::sync_policies::ApproximateTime<
            sensor_msgs::CompressedImage, sensor_msgs::CompressedImage,sensor_msgs::CameraInfo ,zed_interfaces::ObjectsStamped>
            CompressedImageMaskBbSync;

    typedef message_filters::sync_policies::ApproximateTime<
            sensor_msgs::CompressedImage, sensor_msgs::CompressedImage,sensor_msgs::CameraInfo ,
            zed_interfaces::ObjectsStamped, sensor_msgs::PointCloud2>
            CompressedImageMaskBbPclSync;

    typedef message_filters::sync_policies::ApproximateTime<
            sensor_msgs::CompressedImage, sensor_msgs::CompressedImage,sensor_msgs::CameraInfo >
            CompressedImageSync;

    struct DetectedObject {
        bool hasColor = false; // if occluded by other objects, this is false
        ros::Time zedStamp; // zed time
        zed_interfaces::Object zedRawData; // zed raw data (w.r.t cam frame / object callback time)
        cv::Vec3b bgrColor;
        Pose bbPose_w; // simple pose of bounding box ( z forwarding / origin = bb center)
        virtual cv::Vec3b getHsvColor() const;
        virtual bool isPointWithinBB(const Point &pnt_w, bool entireZ, float padding) const;

    };

    struct TrackedObject : public DetectedObject{
        ros::Time clientLastUpdateStamp;
        float p_heightOffset = 0.5;
        float p_smoothingWeight = 0.8;
        int p_queueSize = 50;
        int p_minPntForPoseInference = 10;
        bool velocityComputed = false;
        int beingTrackedAs = -1; // if this becomes a target, this is the index of target (either 0 or 1)

        deque<Point> filteredPointQueue; // z offset and x-forwarding
        Point velocityFiltered_w;
        deque<cv::Vec3b> filteredColorQueue;

        TrackedObject() = default;
        TrackedObject(DetectedObject obj, int targetIdx,
                      float heightOffSetForFilterPose, float smoothing,
                      int queueSize = 30, int minPntForPoseInference = 10);
        void update(const DetectedObject& object);

        cv::Vec3b getHsvColor() const override;
        pcl::PointCloud<pcl::PointXYZRGB> getObservationQueuePCL (string worldFrameId);
        bool getFilteredPoseFromQueue(Pose& pose) const; //! quaternion averaging (position = filterePoint.back)
        bool isPointWithinBB(const Point &pnt_w, bool entireZ, float padding) const override;
        geometry_msgs::PointStamped getLinearPredictionPoint(string worldFrame);
        Point getLinearPredictionPoint() const;
    };

    class Client{
        struct Param{
            bool additionalPcl = false;
            bool filterObject = true;
            bool filterSpeckle = true;

            string worldFrame = "map";

            int nTarget = 2;
            float heightOffsetFromBbCenter = 0.5;
            float targetIdInterval = 0.08;
            float smoothingWeight = 0.7;
            int queueSize = 50;
            int minimumPointsForPoseInference = 10;

            float bbPadding = 0.4;
            int pclStride = 5;
            int maskThicknessX = 10;
            int maskThicknessY = 20;
            bool publishPclAfterTargetLocked= true;

            float MW_color = 1.0; // matching weight
            float MW_location = 1.0;
            float MW_velocity = 1.0;
            float M_colorRejectThreshold = 255;
            float M_colorAcceptThreshold = 100;
            float M_distAcceptThreshold = 0.7;
            float M_distRejectionThreshold = 1.0;

            float speckleSearchRadius = 0.5;
            int speckleNeighbors = 2;
        };

        struct State{
            bool isAllTargetsTracked = false; // matched and queues are updated
            Pose T_wc; // world to cam (optical)
            Pose T_cd; // zed cam to d435 pointcloud frame
            Pose T_cw;
            Pose T_wo; // world to object (x-forwarding)
            ros::Time zedLastCallTime;
            ros::Time clientLastCallTime;
            // visualization
            pcl::PointCloud<pcl::PointXYZRGB> pclObjectsRemoved;
            pcl::PointCloud<pcl::PointXYZRGB> pclFurtherRemoved;
            std::vector<TrackedObject> targetObjects;
            std::vector<DetectedObject> receivedObjects;
            visualization_msgs::MarkerArray targetColors; // only color fields used
        };


    private:
        mutex mutex_;
        Param param;
        State state;

        ros::NodeHandle nh;
        ros::NodeHandle nhGlobal;
        image_transport::ImageTransport it;
        message_filters::Subscriber<sensor_msgs::CompressedImage> * subRgbComp;
        message_filters::Subscriber<sensor_msgs::CompressedImage> * subDepthComp;
        message_filters::Subscriber<zed_interfaces::ObjectsStamped> *subZedOd;
        message_filters::Subscriber<sensor_msgs::CameraInfo> *subCamInfo;
        message_filters::Subscriber<sensor_msgs::PointCloud2> *subAdditionalPcl;
        message_filters::Synchronizer<CompressedImageMaskBbPclSync>* subSyncPcl;
        message_filters::Synchronizer<CompressedImageMaskBbSync>* subSync;
        message_filters::Synchronizer<CompressedImageSync>* subSyncSimple;

        tf::TransformListener* tfListenerPtr;
        tf::TransformBroadcaster* tfBroadcasterPtr;

        image_transport::Publisher pubRgbMaskImg;
        image_transport::Publisher pubDepthMaskImg;

        ros::Publisher pubPointRemoved;
        ros::Publisher pubPointsMasked;
        ros::Publisher pubCurTargetColors; // markerArray where stamp color field is only used
        vector<ros::Publisher> pubObservationFiltered;
        vector<ros::Publisher> pubTargetLinearPrediction;

        ros::Timer timerCaller;
        ros::AsyncSpinner* targetIdSpinnerPtr;
        ros::CallbackQueue observationCallbackQueue;


        float matchingCost (const TrackedObject& priorObj,
                            const DetectedObject& newObj) const;

        void syncSubRoutine(const sensor_msgs::CompressedImageConstPtr &,
                                const sensor_msgs::CompressedImageConstPtr &,
                                const sensor_msgs::CameraInfoConstPtr &,
                                const zed_interfaces::ObjectsStampedConstPtr &,
                                const sensor_msgs::PointCloud2ConstPtr &);

        void zedPclSyncCallback(const sensor_msgs::CompressedImageConstPtr &,
                                const sensor_msgs::CompressedImageConstPtr &,
                                const sensor_msgs::CameraInfoConstPtr &,
                                const zed_interfaces::ObjectsStampedConstPtr &,
                                const sensor_msgs::PointCloud2ConstPtr &);

        void zedSyncCallback(const sensor_msgs::CompressedImageConstPtr &,
                                const sensor_msgs::CompressedImageConstPtr &,
                                const sensor_msgs::CameraInfoConstPtr &,
                                const zed_interfaces::ObjectsStampedConstPtr &
                                );


        void zedSyncCallback(const sensor_msgs::CompressedImageConstPtr &,
                             const sensor_msgs::CompressedImageConstPtr &,
                             const sensor_msgs::CameraInfoConstPtr &);

        // timer callback
        void targetIdCallback(const ros::TimerEvent& event);
    public:
        Client();


    };


}





#endif //ZED2_CLIENT_CLIENT_H
