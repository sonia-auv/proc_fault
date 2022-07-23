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
                imuInfo_subscriber = ros::NodeHandle("~").subscribe("/provider_imu/imu_info", 10, &ControlCheckerModule::imuCallback, this);
                dvlVelocity_subscriber = ros::NodeHandle("~").subscribe("/provider_dvl/dvl_velocity", 10, &ControlCheckerModule::dvlCallback, this);
                faultWarning_publisher = ros::NodeHandle("~").advertise<sonia_common::FaultWarning>("/proc_fault/control_checker_fault_warning", 10);

                monitorThread = std::thread(std::bind(&ControlCheckerModule::monitoringThreadCallback, this));
                monitoringThreadRunning = true;
            }

            ~ControlCheckerModule()
            {
                monitoringThreadRunning = false;
            }

            void dvlCallback(const sonia_common::BodyVelocityDVL &receivedData)
            {   
                std::unique_lock<std::mutex> mlock(dvlMutex, std::defer_lock);
                if(mlock.try_lock() == true)
                {
                    lastDvlVelocityMessage = receivedData;
                    mlock.unlock();
                }
                dvl_timestamp = CommonSoftware::getCurrentTimeMs();
            }

            void imuCallback(const sensor_msgs::Imu &receivedData)
            {   
                imu_timestamp = CommonSoftware::getCurrentTimeMs();
            }

            void monitoringThreadCallback()
            {
                ROS_INFO("ControlCheckerModule Started \n");

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
                bool dvlMessageDetect = true;
                bool dvlBadMessage = true;
                bool imuDetect = true;

                if(!timeDetectionAlgorithm(dvl_timestamp, 200))
                {
                    dvlMessageDetect = false;

                    sonia_common::FaultWarning msg;
                    msg.Module = "DVL";
                    msg.Severity = sonia_common::FaultWarning::Error;
                    msg.Msg = "Dvl does not send data";

                    faultWarning_publisher.publish(msg);
                }


                if(dvlMessageDetect && lastDvlVelocityMessage.xVelBtm == -32.0 && lastDvlVelocityMessage.yVelBtm == -32.0 && lastDvlVelocityMessage.zVelBtm == -32.0 && lastDvlVelocityMessage.eVelBtm == -32.0)
                {
                    dvlBadMessage = false;
                    sonia_common::FaultWarning msg;
                    msg.Module = "DVL";
                    msg.Severity = sonia_common::FaultWarning::Warning;
                    msg.Msg = "Dvl encountered -32 data";

                    faultWarning_publisher.publish(msg);
                }

                if(dvlBadMessage && dvlMessageDetect)
                {
                    sonia_common::FaultWarning msg;
                    msg.Module = "DVL";
                    msg.Severity = sonia_common::FaultWarning::AllGood;
                    msg.Msg = "";

                    faultWarning_publisher.publish(msg);
                }

                if(!timeDetectionAlgorithm(imu_timestamp, 100))
                {
                    imuDetect = false;

                    sonia_common::FaultWarning msg;
                    msg.Module = "IMU";
                    msg.Severity = sonia_common::FaultWarning::Error;
                    msg.Msg = "IMU does not send data";

                    faultWarning_publisher.publish(msg);
                }

                if(imuDetect)
                {
                    sonia_common::FaultWarning msg;
                    msg.Module = "IMU";
                    msg.Severity = sonia_common::FaultWarning::AllGood;
                    msg.Msg = "";

                    faultWarning_publisher.publish(msg);
                }
            }

            bool checkMonitoring() { return true; }

            void rs485Callback(const sonia_common::SendRS485Msg &receivedData) {}
        
        private:

            bool timeDetectionAlgorithm(std::chrono::milliseconds lastTimestamp, unsigned int MaxMs)
            {
                unsigned int diffTimestamp = (CommonSoftware::getCurrentTimeMs() - lastTimestamp).count();
                if(diffTimestamp > MaxMs)
                {
                    return false;
                }

                return true;
            }

            std::chrono::milliseconds dvl_timestamp;
            std::chrono::milliseconds imu_timestamp;

            std::mutex dvlMutex;
            
            sonia_common::BodyVelocityDVL lastDvlVelocityMessage;

            std::thread monitorThread;
            bool monitoringThreadRunning;

            ros::Subscriber dvlVelocity_subscriber;
            ros::Subscriber imuInfo_subscriber;
            
            ros::Publisher faultWarning_publisher;
    };
}

#endif
