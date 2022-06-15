#include <ros/ros.h>
#include <vector>

#include <sonia_common/SendRS485Msg.h>
#include <sonia_common/FaultDetection.h>

#include "proc_fault_node.h"
#include "SoftwareInterface.h"
#include "HardwareInterface.h"
#include "SoftwareImpl.h"
#include "HardwareImpl.h"

namespace proc_fault
{
    ProcFaultNode::ProcFaultNode(const ros::NodeHandlePtr &_nh)
    {
        faultPublisher = _nh->advertise<sonia_common::FaultDetection>("/proc_fault/fault_detection", 10, true);
        rs485Subscriber = _nh->subscribe("/interface_rs485/dataTx", 10, &ProcFaultNode::rs485Callback, this);
        rs485Publisher = _nh->advertise<sonia_common::SendRS485Msg>("/interface_rs485/dataRx", 10);

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
        std::vector<HardwareInterface*> navigationHardwareInterface;

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
        
        if(Configuration::getInstance()->depthEnable)
        {
            navigationSoftwareInterface.push_back(new ProviderDepth());
        }

        procFaultModule.push_back(new Module("Navigation", navigationSoftwareInterface, navigationHardwareInterface));
    }

    void ProcFaultNode::initVision()
    {
        std::vector<SoftwareInterface*> visionSoftwareInterface;
        std::vector<HardwareInterface*> visionHardwareInterface;

        if(Configuration::getInstance()->cameraEnable)
        {
            visionSoftwareInterface.push_back(new ProviderVision());
        }

        if(Configuration::getInstance()->procImageProcessingEnable)
        {
            visionSoftwareInterface.push_back(new ProcImageProcessing());
        }

        if(Configuration::getInstance()->procDetectionEnable)
        {
            visionSoftwareInterface.push_back(new ProcDetection());
        }

        procFaultModule.push_back(new Module("Vision", visionSoftwareInterface, visionHardwareInterface));
    }

    void ProcFaultNode::initMapping()
    {
        std::vector<SoftwareInterface*> mappingSoftwareInterface;
        std::vector<HardwareInterface*> mappingHardwareInterface;

        if(Configuration::getInstance()->sonarEnable)
        {
            mappingSoftwareInterface.push_back(new ProviderSonar());
        }

        if(Configuration::getInstance()->mappingEnable)
        {
            //mappingSoftwareInterface.push_back(new ProcMapping());
        }

        procFaultModule.push_back(new Module("Mapping", mappingSoftwareInterface, mappingHardwareInterface));
    }

    void ProcFaultNode::initHydro()
    {
        std::vector<SoftwareInterface*> hydroSoftwareInterface;
        std::vector<HardwareInterface*> hydroHardwareInterface;

        if(Configuration::getInstance()->providerHydroEnable)
        {
            hydroSoftwareInterface.push_back(new ProviderHydrophone());
        }

        if(Configuration::getInstance()->procHydroEnable)
        {
            hydroSoftwareInterface.push_back(new ProcHydrophone());
        }

        procFaultModule.push_back(new Module("Hydro", hydroSoftwareInterface, hydroHardwareInterface));
    }

    void ProcFaultNode::initIo()
    {
        std::vector<SoftwareInterface*> ioSoftwareInterface;
        std::vector<HardwareInterface*> ioHardwareInterface;

        if(Configuration::getInstance()->providerActuatorEnable)
        {
            ioSoftwareInterface.push_back(new ProviderActuator());
        }

        if(Configuration::getInstance()->procActuatorEnable)
        {
            ioSoftwareInterface.push_back(new ProcActuator());
        }

        procFaultModule.push_back(new Module("Io", ioSoftwareInterface, ioHardwareInterface));
    }

    void ProcFaultNode::initUnderwaterCom()
    {
        std::vector<SoftwareInterface*> underwaterComSoftwareInterface;
        std::vector<HardwareInterface*> underwaterComHardwareInterface;

        if(Configuration::getInstance()->providerUnderwaterComEnable)
        {
            underwaterComSoftwareInterface.push_back(new ProviderCom());
        }
        
        procFaultModule.push_back(new Module("Underwater Com", underwaterComSoftwareInterface, underwaterComHardwareInterface));
    }

    void ProcFaultNode::initPower()
    {
        std::vector<SoftwareInterface*> powerSoftwareInterface;
        std::vector<HardwareInterface*> powerHardwareInterface;

        if(Configuration::getInstance()->powerEnable)
        {
            powerSoftwareInterface.push_back(new ProviderPower());
        }

        if(Configuration::getInstance()->providerThrusterEnable)
        {
            powerSoftwareInterface.push_back(new ProviderThruster());
        }

        if(Configuration::getInstance()->boardPowerSupplyEnable)
        {
            powerHardwareInterface.push_back(new BoardPowerSupply(&rs485Publisher));
        }
        
        procFaultModule.push_back(new Module("Power", powerSoftwareInterface, powerHardwareInterface));
    }

    void ProcFaultNode::initInternalCom()
    {
        std::vector<SoftwareInterface*> internalComSoftwareInterface;
        std::vector<HardwareInterface*> internalComHardwareInterface;

        if(Configuration::getInstance()->interfaceEnable)
        {
            internalComSoftwareInterface.push_back(new InterfaceRs485());
        }
        
        procFaultModule.push_back(new Module("Internal Com", internalComSoftwareInterface, internalComHardwareInterface));
    }

    void ProcFaultNode::rs485Callback(const sonia_common::SendRS485Msg &receivedData)
    {
        for(Module* module : procFaultModule)
        {
            module->rs485Callback(receivedData);
        }
    }

    void ProcFaultNode::spin()
    {
        ros::Rate r(5);
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