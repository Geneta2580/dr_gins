#include <ros/ros.h>
#include <common/dr_gins_interface.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "dr_gins_node");

    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    dr_gins::DrGinsInterface interface(nh, private_nh);

    ros::spin();

    return 0;
}
