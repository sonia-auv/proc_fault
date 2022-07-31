#ifndef PROC_FAULT_HARDWARE_INTERFACE_H
#define PROC_FAULT_HARDWARE_INTERFACE_H

#include <sonia_common/SendRS485Msg.h>

#include <string>

namespace proc_fault
{
    class HardwareInterface 
    {
        public:
            virtual ~HardwareInterface()
            {

            }
            
            virtual bool detection() = 0;
            virtual bool correction() = 0;

            virtual void rs485Publish() = 0;
            virtual void rs485Callback(const sonia_common::SendRS485Msg &receivedData) = 0;

            virtual void resetErrorNotification()
            {
                if(!newError)
                {
                    ROS_WARN("Hardware: %s as recovered", getName().c_str());
                    newError = true;
                }
            }

            virtual void printErrorNotification()
            {
                if(newError)
                {
                    ROS_ERROR("Hardware: %s stop receiving keep alive", getName().c_str());
                    newError = false;
                }
            }

            virtual std::string getName() = 0;

        private:
            bool newError = true;
    };
}

#endif
