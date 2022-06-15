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

            virtual std::string getName() = 0;
    };
}

#endif