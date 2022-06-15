#ifndef PROC_FAULT_NODE_H
#define PROC_FAULT_NODE_H

#include <ros/ros.h>
#include "Module.h"

namespace proc_fault
{
    class ProcFaultNode
    {
        public:

            ProcFaultNode(const ros::NodeHandlePtr &_nh);
            ~ProcFaultNode();

            void spin();

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

            std::vector<Module*> procFaultModule;
            ros::Publisher faultPublisher;

            ros::Subscriber rs485Subscriber;
            ros::Publisher rs485Publisher;
    };
}

#endif