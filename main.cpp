//
// Created by jbs on 21. 6. 3..
//
#include <zed2_client/Client.h>

using namespace zed_client;
int main(int argc, char ** argv){
    ros::init(argc,argv,"zed_client_node");
    Client client;
    ros::spin();
    return 0;
}