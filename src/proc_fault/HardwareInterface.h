#ifndef PROC_FAULT_HARDWARE_INTERFACE_H
#define PROC_FAULT_HARDWARE_INTERFACE_H

namespace procFault
{
    class HardwareInterface 
    {
        public:
            virtual bool Detection();
            virtual bool Correction();
    };
}

#endif