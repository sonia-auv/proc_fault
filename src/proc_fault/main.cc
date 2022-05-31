#include <ros/ros.h>
#include "proc_fault_node.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "proc_fault");

    ros::NodeHandlePtr nh(new ros::NodeHandle("~"));
    proc_fault::ProcFaultNode fault_node{nh};
    fault_node.spin();
}
