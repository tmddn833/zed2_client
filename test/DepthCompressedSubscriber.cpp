//
// Created by jbs on 21. 6. 3..
//


#include <zed2_client/Utils.h>

namespace enc = sensor_msgs::image_encodings;
using namespace zed_client;


void depthCallback(const sensor_msgs::CompressedImageConstPtr& depthPtr){
    cv::Mat decompressed;
    pngDecompressDepth(depthPtr,decompressed);
    imshow("depth decompressed",decompressed);
    cv::waitKey(1);
}
void rgbCallback(const sensor_msgs::CompressedImageConstPtr& rgbPtr){

    cv::Mat decompressed;
    jpegDecompressRgb(rgbPtr,decompressed);

    imshow("rgb decompressed",decompressed);
    cv::waitKey(1);


}
int main(int argc, char** argv){

    ros::init(argc,argv,"depth_subscriber");
    ros::NodeHandle nh("~");
    image_transport::ImageTransport it(nh);
    ros::Subscriber subDepth = nh.subscribe("/zed2/zed_node/depth/depth_registered/compressedDepth",1, depthCallback);
    ros::Subscriber subRgb = nh.subscribe("/zed2/zed_node/rgb/image_rect_color/compressed",1,rgbCallback);

    ros::spin();
    return 0;
}
