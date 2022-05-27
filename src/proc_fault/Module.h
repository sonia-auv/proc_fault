#ifndef PROC_FAULT_MODULE_H
#define PROC_FAULT_MODULE_H

#include <vector>

#include "SoftwareInterface.h"

namespace procFault
{
    class Module 
    {
        public:
            Module(std::string _moduleName, std::vector<SoftwareInterface*> _softwareInterfaceArray)
            {
                softwareInterfaceArray = _softwareInterfaceArray;
                moduleName = _moduleName;

                monitorThreadRunning = true;
                monitoringResult = true;
                monitorThread = std::thread(std::bind(&Module::monitoringThreadCallback, this));
            }

            ~Module()
            {
                std::unique_lock<std::mutex> mlock(ArraysMutex);

                monitorThreadRunning = false;
                for(SoftwareInterface* soft : softwareInterfaceArray)
                {
                    delete soft;
                }

                softwareInterfaceArray.clear();
            }

            void monitoringThreadCallback()
            {
                ROS_INFO("MODULE: %s Started \n", moduleName);
                // wait for the module startup
                ros::Duration(60).sleep();
                ROS_INFO("MODULE: %s is Currently monitoring \n", moduleName);

                while(monitorThreadRunning)
                {
                    //sleep 1 sec
                    ros::Duration(1).sleep();
                    self.monitor();
                }
            }

            void monitor()
            {
                // check every detection
                std::unique_lock<std::mutex> mlock(ArraysMutex);
                for(SoftwareInterface* soft : softwareInterfaceArray)
                {
                    monitoringResult &= soft->detection();
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
    };
}

#endif