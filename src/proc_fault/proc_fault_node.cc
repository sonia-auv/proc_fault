#include <ros/ros.h>
#include <vector>

#include "proc_fault_node.h"
#include "SoftwareInterface.h"
#include "SoftwareImpl.h"

namespace proc_fault
{
    ProcFaultNode::ProcFaultNode(const ros::NodeHandlePtr &_nh)
    {
        std::vector<SoftwareInterface*> controlSoftwareInterface;

        controlSoftwareInterface.push_back(new ProviderImu());
        
        Module* controlModule = new Module("Control", controlSoftwareInterface);

        procFaultModule.push_back(controlModule);
    }

    ProcFaultNode::~ProcFaultNode()
    {
        for(Module* module : procFaultModule)
        {
            delete module;
        }

        procFaultModule.clear();

    }

    void ProcFaultNode::spin()
    {
        bool test = procFaultModule[0]->checkMonitoring();
        ROS_INFO("spin happened with the boolean %d\n", test);
    }
}