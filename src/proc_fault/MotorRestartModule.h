#ifndef MOTOR_RESTART_MODULE_H
#define MOTOR_RESTART_MODULE_H

#include <vector>
#include <thread>
#include <mutex>

#include <sonia_common/SendRS485Msg.h>

namespace proc_fault
{
    class MotorRestartModule : public ModuleInterface
    {
        public:
            MotorRestartModule()
            {
                provider_power_motor_feedback_topic = ros::NodeHandle("~").subscribe("/provider_power/motor_feedback", 10, &MotorRestartModule::motorFeedback, this);
                activate_motor_publisher = ros::NodeHandle("~").advertise<std_msgs::UInt8MultiArray>("/provider_power/activate_motor", 10);

                motor_feedback_publisher = ros::NodeHandle("~").advertise<std_msgs::UInt8MultiArray>("/proc_fault/motor_feedback", 10);

                monitorThread = std::thread(std::bind(&MotorRestartModule::monitoringThreadCallback, this));
                monitoringThreadRunning = true;
            }

            ~MotorRestartModule()
            {
                monitoringThreadRunning = false;
            }

            void motorFeedback(const std_msgs::UInt8MultiArray::ConstPtr &receivedData)
            {   
                std::unique_lock<std::mutex> mlock(motorMutex, std::defer_lock);
                if(mlock.try_lock() == true)
                {
                    for(unsigned long int i = 0; i < receivedData->data.size(); ++i)
                    {
                        motorStatus[i] = receivedData->data[i];
                    }

                    mlock.unlock();
                }
            }

            void monitoringThreadCallback()
            {
                ROS_INFO("MotorRestartModule Started \n");

                while(monitoringThreadRunning)
                {
                    //sleep 1 sec
                    ros::Duration(1).sleep();
                    this->monitor();
                }
            }
            
            void monitor()
            {
                bool motor_error_found = false;
                std_msgs::UInt8MultiArray motorMsg;
                std_msgs::UInt8MultiArray feedbackMsg;

                std::unique_lock<std::mutex> mlock(motorMutex);

                for(unsigned long int i = 0; i < feedbackMsg.data.size(); ++i)
                {
                    if(feedbackMsg.data[i] >= 2)
                    {
                        if(!motorAlreadyReseted[i])
                        {
                            motor_error_found = true;
                            motorStatus[i] = 4;
                            motorMsg.data.push_back(0);
                            motorAlreadyReseted[i] = true;
                        }
                        else
                        {
                            motorMsg.data.push_back(1);
                        }
                    }
                    else
                    {
                        motorMsg.data.push_back(1);
                    }

                    feedbackMsg.data.push_back(motorStatus[i]);
                }

                if(motor_error_found)
                {
                    activate_motor_publisher.publish(motorMsg);
                    motor_feedback_publisher.publish(feedbackMsg);

                    ros::Duration(5).sleep();

                    std::fill(motorMsg.data.begin(),  motorMsg.data.end(), 1);
                    activate_motor_publisher.publish(motorMsg);
                }
                else
                {
                    motor_feedback_publisher.publish(feedbackMsg);
                }

            }

            bool checkMonitoring() { return true; }

            void rs485Callback(const sonia_common::SendRS485Msg &receivedData) {}
        
        private:

            std::mutex motorMutex;
            
            std::array<bool,8> motorAlreadyReseted = {false};
            std::array<int,8> motorStatus = {0};
            std::thread monitorThread;
            bool monitoringThreadRunning;
            

            ros::Subscriber provider_power_motor_feedback_topic;
            
            ros::Publisher activate_motor_publisher;
            ros::Publisher motor_feedback_publisher;
    };
}

#endif