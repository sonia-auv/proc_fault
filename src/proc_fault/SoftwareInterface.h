#ifndef PROC_FAULT_SOFTWARE_INTERFACE_H
#define PROC_FAULT_SOFTWARE_INTERFACE_H

namespace proc_fault
{
    class SoftwareInterface 
    {
        public:
            virtual ~SoftwareInterface()
            {

            }
            
            virtual bool detection() = 0;
            virtual bool correction() = 0;
    };
}

#endif
