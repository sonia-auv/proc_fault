#ifndef PROC_FAULT_HARDWARE_INTERFACE_H
#define PROC_FAULT_HARDWARE_INTERFACE_H

namespace proc_fault
{
    class HardwareInterface 
    {
        public:
            virtual ~HardwareInterface();
            virtual bool Detection();
            virtual bool Correction();
    };
}

#endif