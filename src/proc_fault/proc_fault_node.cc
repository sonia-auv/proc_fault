#include <ros/ros.h>
#include <vector>

#include <std_msgs/UInt8MultiArray.h>

#include <sonia_common/SendRS485Msg.h>
#include <sonia_common/FaultDetection.h>

#include "proc_fault_node.h"
#include "SoftwareInterface.h"
#include "HardwareInterface.h"
#include "SoftwareImpl.h"
#include "HardwareImpl.h"

#include "MissionCheckerModule.h"
#include "ControlCheckerModule.h"
#include "ModuleInterface.h"
#include "MotorRestartModule.h"


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


        if(Configuration::getInstance()->enableMotorResetModule)
        {
            procFaultModule["MotorRestartModule"] = new MotorRestartModule();
        }

        if(Configuration::getInstance()->enableControlCheckerModule)
        {
            procFaultModule["ControlCheckerModule"] = new ControlCheckerModule();
        }
    }

    ProcFaultNode::~ProcFaultNode()
    {
        for(std::pair<std::string, ModuleInterface*> module_pair : procFaultModule)
        {
            delete module_pair.second;
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

        if(Configuration::getInstance()->boardEscEnable)
        {
            navigationHardwareInterface.push_back(new BoardEsc(&rs485Publisher, sonia_common::SendRS485Msg::SLAVE_ESC, "Board Esc", Configuration::getInstance()->boardEscTimestampsMs));
        }

        procFaultModule[NavigationName] = new MissionCheckerModule(NavigationName, navigationSoftwareInterface, navigationHardwareInterface);
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

        procFaultModule[VisionName] = new MissionCheckerModule(VisionName, visionSoftwareInterface, visionHardwareInterface);
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

        procFaultModule[MappingName] = new MissionCheckerModule(MappingName, mappingSoftwareInterface, mappingHardwareInterface);
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

        procFaultModule[HydroName] = new MissionCheckerModule(HydroName, hydroSoftwareInterface, hydroHardwareInterface);
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

        if(Configuration::getInstance()->boardIoEnable)
        {
            ioHardwareInterface.push_back(new BoardKillMission(&rs485Publisher, sonia_common::SendRS485Msg::SLAVE_IO, "Io Board", Configuration::getInstance()->boardIoTimestampsMs));
        }

        procFaultModule[IoName] = new MissionCheckerModule(IoName, ioSoftwareInterface, ioHardwareInterface);
    }

    void ProcFaultNode::initUnderwaterCom()
    {
        std::vector<SoftwareInterface*> underwaterComSoftwareInterface;
        std::vector<HardwareInterface*> underwaterComHardwareInterface;

        if(Configuration::getInstance()->providerUnderwaterComEnable)
        {
            underwaterComSoftwareInterface.push_back(new ProviderCom());
        }
        
        procFaultModule[UnderwaterName] = new MissionCheckerModule(UnderwaterName, underwaterComSoftwareInterface, underwaterComHardwareInterface);
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
            powerHardwareInterface.push_back(new BoardPowerSupply(&rs485Publisher, sonia_common::SendRS485Msg::SLAVE_PWR_MANAGEMENT, "Power Supply", Configuration::getInstance()->boardPowerSupplyTimestampsMs));
        }
        
        procFaultModule[PowerName] = new MissionCheckerModule(PowerName, powerSoftwareInterface, powerHardwareInterface);
    }

    void ProcFaultNode::initInternalCom()
    {
        std::vector<SoftwareInterface*> internalComSoftwareInterface;
        std::vector<HardwareInterface*> internalComHardwareInterface;

        if(Configuration::getInstance()->interfaceEnable)
        {
            internalComSoftwareInterface.push_back(new InterfaceRs485());
        }

        if(Configuration::getInstance()->boardKillMissionEnable)
        {
            internalComHardwareInterface.push_back(new BoardKillMission(&rs485Publisher, sonia_common::SendRS485Msg::SLAVE_KILLMISSION, "Kill Mission Switch", Configuration::getInstance()->boardKillMissionTimestampsMs));
        }
        
        procFaultModule[InternalName] = new MissionCheckerModule(InternalName, internalComSoftwareInterface, internalComHardwareInterface);
    }

    void ProcFaultNode::rs485Callback(const sonia_common::SendRS485Msg &receivedData)
    {
        for(std::pair<std::string, ModuleInterface*> module_pair : procFaultModule)
        {
            module_pair.second->rs485Callback(receivedData);
        }
    }

    void ProcFaultNode::spin()
    {
        ros::Rate r(20);
        while(ros::ok())
        {
            ros::spinOnce();

            sonia_common::FaultDetection msg;
            msg.navigation = procFaultModule[NavigationName]->checkMonitoring();
            msg.vision = procFaultModule[VisionName]->checkMonitoring();
            msg.mapping = procFaultModule[MappingName]->checkMonitoring();
            msg.hydro = procFaultModule[HydroName]->checkMonitoring();
            msg.io = procFaultModule[IoName]->checkMonitoring();
            msg.underwater_com = procFaultModule[UnderwaterName]->checkMonitoring();
            msg.power = procFaultModule[PowerName]->checkMonitoring();
            msg.internal_com = procFaultModule[InternalName]->checkMonitoring();

            faultPublisher.publish(msg);

            r.sleep();
        }
    }
}