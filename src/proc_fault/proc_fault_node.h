#ifndef PROC_FAULT_NODE_H
#define PROC_FAULT_NODE_H

#include <ros/ros.h>
#include "ModuleInterface.h"

namespace proc_fault
{
    class ProcFaultNode
    {
        public:

            ProcFaultNode(const ros::NodeHandlePtr &_nh);
            ~ProcFaultNode();

            void spin();

            const std::string NavigationName = "Navigation";
            const std::string VisionName = "Vision";
            const std::string MappingName = "Mapping";
            const std::string HydroName = "Hydro";
            const std::string IoName = "Io";
            const std::string UnderwaterName = "Underwater Com";
            const std::string PowerName = "Power";
            const std::string InternalName = "Internal Com";

        private:
            void initNavigation();
            void initVision();
            void initMapping();
            void initHydro();
            void initIo();
            void initUnderwaterCom();
            void initPower();
            void initInternalCom();

            void rs485Callback(const sonia_common::SendRS485Msg &receivedData);

            std::map<std::string, ModuleInterface*> procFaultModule;
            ros::Publisher faultPublisher;

            ros::Subscriber rs485Subscriber;
            ros::Publisher rs485Publisher;
    };
}

#endif