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
        initVision();
        initMapping();
        initHydro();
        initIo();
        initUnderwaterCom();
        initPower();
        initInternalCom();
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
        std::vector<SoftwareInterface*> mappingSoftwareInterface;

        if(Configuration::getInstance()->sonarEnable)
        {
            mappingSoftwareInterface.push_back(new ProviderSonar());
        }

        procFaultModule.push_back(new Module("Mapping", mappingSoftwareInterface));
    }

    void ProcFaultNode::initHydro()
    {
        std::vector<SoftwareInterface*> hydroSoftwareInterface;

        if(Configuration::getInstance()->providerHydroEnable)
        {
            hydroSoftwareInterface.push_back(new ProviderHydrophone());
        }

        if(Configuration::getInstance()->procHydroEnable)
        {
            hydroSoftwareInterface.push_back(new ProcHydrophone());
        }

        procFaultModule.push_back(new Module("Hydro", hydroSoftwareInterface));
    }

    void ProcFaultNode::initIo()
    {
        std::vector<SoftwareInterface*> ioSoftwareInterface;



        procFaultModule.push_back(new Module("Io", ioSoftwareInterface));
    }

    void ProcFaultNode::initUnderwaterCom()
    {
        std::vector<SoftwareInterface*> underwaterComSoftwareInterface;


        
        procFaultModule.push_back(new Module("Underwater Com", underwaterComSoftwareInterface));
    }

    void ProcFaultNode::initPower()
    {
        std::vector<SoftwareInterface*> powerSoftwareInterface;

        if(Configuration::getInstance()->powerEnable)
        {
            powerSoftwareInterface.push_back(new ProviderPower());
        }
        
        procFaultModule.push_back(new Module("Power", powerSoftwareInterface));
    }

    void ProcFaultNode::initInternalCom()
    {
        std::vector<SoftwareInterface*> internalComSoftwareInterface;

        if(Configuration::getInstance()->interfaceEnable)
        {
            internalComSoftwareInterface.push_back(new InterfaceRs485());
        }
        
        procFaultModule.push_back(new Module("Internal Com", internalComSoftwareInterface));
    }

    void ProcFaultNode::spin()
    {
        ros::Rate r(20);
        while(ros::ok())
        {
            ros::spinOnce();

            sonia_common::FaultDetection msg;
            msg.navigation = procFaultModule[0]->checkMonitoring();
            msg.vision = procFaultModule[1]->checkMonitoring();
            msg.mapping = procFaultModule[2]->checkMonitoring();
            msg.io = procFaultModule[3]->checkMonitoring();
            msg.underwater_com = procFaultModule[4]->checkMonitoring();
            msg.power = procFaultModule[5]->checkMonitoring();
            msg.internal_com = procFaultModule[6]->checkMonitoring();

            faultPublisher.publish(msg);

            r.sleep();
        }
    }
}