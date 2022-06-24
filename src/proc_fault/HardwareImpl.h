#ifndef PROC_FAULT_HARDWARE_IMPLEMENTATION_H
#define PROC_FAULT_HARDWARE_IMPLEMENTATION_H

#include <sonia_common/SendRS485Msg.h>

#include <ros/ros.h>
#include <chrono>

#include "HardwareInterface.h"
#include "Configuration.h"

namespace proc_fault
{
    class CommonHardware
    {
        public:
            static std::chrono::milliseconds getCurrentTimeMs()
            {
                return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());
            }

            static bool timeDetectionAlgorithm(std::chrono::milliseconds lastTimestamp, unsigned int MaxMs)
            {
                unsigned int diffTimestamp = (CommonHardware::getCurrentTimeMs() - lastTimestamp).count();
                if(diffTimestamp > MaxMs)
                {
                    return false;
                }

                return true;
            }
    };

    class BoardPowerSupply : public HardwareInterface
    {
        public:
            BoardPowerSupply(ros::Publisher* _rs485Topic)
            {
                rs485Topic = _rs485Topic;
            }

            void rs485Callback(const sonia_common::SendRS485Msg &receivedData)
            {
                if(receivedData.slave == slaveId)
                {
                    timestamp = CommonHardware::getCurrentTimeMs();
                }
            }

            void rs485Publish()
            {
                sonia_common::SendRS485Msg msg;
                msg.slave = slaveId;

                // TODO: replace it by the good value
                msg.cmd = 30;

                rs485Topic->publish(msg);
            }

            bool detection()
            {
                return CommonSoftware::timeDetectionAlgorithm(timestamp, Configuration::getInstance()->boardPowerSupplyMs);
            }

            bool correction()
            {
                return true;
            }

            std::string getName()
            {
                return name;
            }

        private:
            uint8_t slaveId = sonia_common::SendRS485Msg::SLAVE_PWR_MANAGEMENT;
            ros::Publisher* rs485Topic;

            std::string name = "Power Supply";
            std::chrono::milliseconds timestamp;
    };
}

#endif