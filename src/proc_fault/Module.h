#ifndef PROC_FAULT_MODULE_H
#define PROC_FAULT_MODULE_H

#include <vector>
#include <thread>
#include <mutex>

#include "SoftwareInterface.h"
#include "Configuration.h"

namespace proc_fault
{
    class Module 
    {
        public:
            Module(std::string _moduleName, std::vector<SoftwareInterface*> _softwareInterfaceArray)
            {
                softwareInterfaceArray = _softwareInterfaceArray;
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

            void monitoringThreadCallback()
            {
                ROS_INFO("MODULE: %s Started \n", moduleName.c_str());
                // wait for the module startup
                ros::Duration(Configuration::getInstance()->initialNodeSleepSecond).sleep();
                ROS_INFO("MODULE: %s is Currently monitoring \n", moduleName.c_str());

                while(monitoringThreadRunning)
                {
                    //sleep 1 sec
                    ros::Duration(1).sleep();
                    this->monitor();
                }
            }

            void monitor()
            {
                // check every detection
                bool tempMonitoring = true;
                std::unique_lock<std::mutex> mlock(ArraysMutex);
                for(SoftwareInterface* soft : softwareInterfaceArray)
                {
                    tempMonitoring &= soft->detection();
                }
                monitoringResult = tempMonitoring;
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
    };
}

#endif