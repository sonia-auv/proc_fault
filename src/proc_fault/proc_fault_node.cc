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
        initNavigation();   
    }

    ProcFaultNode::~ProcFaultNode()
    {
        for(Module* module : procFaultModule)
        {
            delete module;
        }

        procFaultModule.clear();

    }

    void ProcFaultNode::initNavigation()
    {
        std::vector<SoftwareInterface*> navigationSoftwareInterface;

        if(Configuration::getInstance()->controlEnable)
        {
            navigationSoftwareInterface.push_back(new ProcControl());
        }
        
        if(Configuration::getInstance()->imuEnable)
        {
            navigationSoftwareInterface.push_back(new ProviderImu());
        }
        
        if(Configuration::getInstance()->dvlEnable)
        {
            navigationSoftwareInterface.push_back(new ProviderDvl());
        }
        
        if(Configuration::getInstance()->dvlEnable)
        {
            navigationSoftwareInterface.push_back(new ProviderDepth());
        }

        procFaultModule.push_back(new Module("Navigation", navigationSoftwareInterface));
    }

    void ProcFaultNode::initVision()
    {
        std::vector<SoftwareInterface*> visionSoftwareInterface;

        if(Configuration::getInstance()->cameraEnable)
        {
            visionSoftwareInterface.push_back(new ProviderVision());
        }

        procFaultModule.push_back(new Module("Vision", visionSoftwareInterface));
    }

    void ProcFaultNode::initMapping()
    {

    }

    void ProcFaultNode::initHydro()
    {
        
    }

    void ProcFaultNode::initIo()
    {
        
    }

    void ProcFaultNode::initUnderwaterCom()
    {
        
    }

    void ProcFaultNode::initPower()
    {
        
    }

    void ProcFaultNode::initInternalCom()
    {
        
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