#ifndef PROC_FAULT_MODULE_H
#define PROC_FAULT_MODULE_H

#include <sonia_common/SendRS485Msg.h>

#include <vector>
#include <thread>
#include <mutex>

#include "SoftwareInterface.h"
#include "HardwareInterface.h"
#include "Configuration.h"

namespace proc_fault
{
    class Module 
    {
        public:
            Module(std::string _moduleName, std::vector<SoftwareInterface*> _softwareInterfaceArray, std::vector<HardwareInterface*> _hardwareInterfaceArray)
            {
                softwareInterfaceArray = _softwareInterfaceArray;
                hardwareInterfaceArray = _hardwareInterfaceArray;
                moduleName = _moduleName;

                monitoringThreadRunning = true;
                monitoringResult = true;
                monitorThread = std::thread(std::bind(&Module::monitoringThreadCallback, this));
            }

            ~Module()
            {
                std::unique_lock<std::mutex> mlock(ArraysMutex);

                monitoringThreadRunning = false;
                for(SoftwareInterface* soft : softwareInterfaceArray)
                {
                    delete soft;
                }

                softwareInterfaceArray.clear();
            }

            void rs485Callback(const sonia_common::SendRS485Msg &receivedData)
            {
                for(HardwareInterface* hard : hardwareInterfaceArray)
                {
                    hard->rs485Callback(receivedData);
                }
            }

            void monitoringThreadCallback()
            {
                ROS_INFO("MODULE: %s Started \n", moduleName.c_str());
                // wait for the module startup
                ros::Duration(Configuration::getInstance()->initialNodeSleepSecond).sleep();
                ROS_INFO("MODULE: %s is Currently monitoring \n", moduleName.c_str());

                while(monitoringThreadRunning)
                {
                    //sleep 1 sec
                    ros::Duration(Configuration::getInstance()->loopSleepTimeSecond).sleep();
                    this->monitor();
                    this->publishRs485();
                }
            }

            void monitor()
            {
                // check every detection
                bool tempMonitoring = true;
                std::unique_lock<std::mutex> mlock(ArraysMutex);
                for(SoftwareInterface* soft : softwareInterfaceArray)
                {
                    bool detection = soft->detection();
                    tempMonitoring &= detection;

                    if(!detection)
                    {
                        soft->printErrorNotification();
                    }
                    else
                    {
                        soft->resetErrorNotification();
                    }
                }
                monitoringResult = tempMonitoring;
            }

            void publishRs485()
            {
                for(HardwareInterface* hard : hardwareInterfaceArray)
                {
                    hard->rs485Publish();
                }
            }

            bool checkMonitoring()
            {
                return monitoringResult;
            }
        
        private:
            bool monitoringResult;
            bool monitoringThreadRunning;

            std::string moduleName;
            std::thread monitorThread;
            std::mutex ArraysMutex;
            std::vector<SoftwareInterface*> softwareInterfaceArray;
            std::vector<HardwareInterface*> hardwareInterfaceArray;
    };
}

#endif