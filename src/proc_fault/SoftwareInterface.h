#ifndef PROC_FAULT_SOFTWARE_INTERFACE_H
#define PROC_FAULT_SOFTWARE_INTERFACE_H

#include <string>

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

            virtual std::string getName() = 0;
    };
}

#endif
