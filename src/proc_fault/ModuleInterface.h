#ifndef MODULE_INTERFACE_H
#define MODULE_INTERFACE_H

#include <sonia_common/SendRS485Msg.h>

namespace proc_fault
{
    class ModuleInterface 
    {
        public:
            virtual ~ModuleInterface()
            {
                
            }
            
            virtual bool checkMonitoring() = 0;
            virtual void rs485Callback(const sonia_common::SendRS485Msg &receivedData) = 0;
    };
}

#endif