#ifndef PROC_FAULT_SOFTWARE_INTERFACE_H
#define PROC_FAULT_SOFTWARE_INTERFACE_H

namespace procFault
{
    class SoftwareInterface 
    {
        public:
            virtual bool detection();
            virtual bool correction();
    };
}

#endif
