//
// Created by jbs on 21. 5. 27..
//

#include <ros/ros.h>
#include <rosgraph_msgs/Clock.h>

int main(int argc, char** argv){
    ros::init(argc,argv,"clock_publisher");
    ros::NodeHandle nh("~");
    ros::Publisher pubClock = nh.advertise<rosgraph_msgs::Clock>("/clock",1);
    rosgraph_msgs::Clock clock;
    while(ros::ok()){
        clock.clock = ros::Time::now();
        pubClock.publish(clock);
        ros::Rate(30).sleep();
    }
    return 0;
}