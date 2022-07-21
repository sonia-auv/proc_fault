#ifndef MISSION_CHECKER_MODULE_H
#define MISSION_CHECKER_MODULE_H

#include <vector>
#include <thread>
#include <mutex>

#include <sonia_common/BodyVelocityDVL.h>
#include <sonia_common/FaultWarning.h>

namespace proc_fault
{
    class ControlCheckerModule : public ModuleInterface
    {
        public:
            ControlCheckerModule()
            {
                dvlVelocity_subscriber = ros::NodeHandle("~").subscribe("/provider_dvl/dvl_velocity", 10, &ControlCheckerModule::dvlFeedback, this);
                faultWarning_publisher = ros::NodeHandle("~").advertise<sonia_common::FaultWarning>("/proc_fault/control_checker_fault_warning", 10);

                monitorThread = std::thread(std::bind(&ControlCheckerModule::monitoringThreadCallback, this));
                monitoringThreadRunning = true;
            }

            ~ControlCheckerModule()
            {
                monitoringThreadRunning = false;
            }

            void dvlFeedback(const sonia_common::BodyVelocityDVL &receivedData)
            {   
                std::unique_lock<std::mutex> mlock(dvlMutex, std::defer_lock);
                if(mlock.try_lock() == true)
                {
                    lastDvlVelocityMessage = receivedData;
                    mlock.unlock();
                }
            }

            void monitoringThreadCallback()
            {
                ROS_INFO("MotorRestartModule Started \n");

                while(monitoringThreadRunning)
                {
                    //sleep 500 msec
                    ros::Duration(0.5).sleep();
                    this->monitor();
                }
            }
            
            void monitor()
            {
                std::unique_lock<std::mutex> mlock(dvlMutex);


                if(lastDvlVelocityMessage.xVelBtm == -32.0 && lastDvlVelocityMessage.yVelBtm == -32.0 && lastDvlVelocityMessage.zVelBtm == -32.0 && lastDvlVelocityMessage.eVelBtm == -32.0)
                {
                    sonia_common::FaultWarning msg;
                    msg.Module = "DVL";
                    msg.Severity = sonia_common::FaultWarning::Warning;
                    msg.Msg = "Dvl encountered -32 data";

                    faultWarning_publisher.publish(msg);
                }
            }

            bool checkMonitoring() { return true; }

            void rs485Callback(const sonia_common::SendRS485Msg &receivedData) {}
        
        private:

            std::mutex dvlMutex;
            
            
            sonia_common::BodyVelocityDVL lastDvlVelocityMessage;

            std::thread monitorThread;
            bool monitoringThreadRunning;

            ros::Subscriber dvlVelocity_subscriber;
            
            ros::Publisher faultWarning_publisher;
    };
}

#endif