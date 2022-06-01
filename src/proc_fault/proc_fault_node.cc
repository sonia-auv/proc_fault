#include <ros/ros.h>
#include <vector>

#include <sonia_common/FaultDetection.h>

#include "proc_fault_node.h"
#include "SoftwareInterface.h"
#include "SoftwareImpl.h"

namespace proc_fault
{
    ProcFaultNode::ProcFaultNode(const ros::NodeHandlePtr &_nh)
    {
        faultPublisher = _nh->advertise<sonia_common::FaultDetection>("/proc_fault/fault_detection", 10, true);

        std::vector<SoftwareInterface*> navigationSoftwareInterface;

        navigationSoftwareInterface.push_back(new ProviderImu());
        
        Module* navigationModule = new Module("Navigation", navigationSoftwareInterface);

        procFaultModule.push_back(navigationModule);
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
         ros::Rate r(20);
        while(ros::ok())
        {
            ros::spinOnce();

            sonia_common::FaultDetection msg;
            msg.navigation = procFaultModule[0]->checkMonitoring();

            faultPublisher.publish(msg);

            r.sleep();
        }
    }
}