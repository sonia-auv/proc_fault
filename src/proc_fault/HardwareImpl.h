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

    class Rs485KeepAliveBoard : public HardwareInterface
    {
         public:
            Rs485KeepAliveBoard(ros::Publisher* _rs485Topic, uint8_t _slaveId, std::string _name, unsigned int _target_timestamp)
            {
                rs485Topic = _rs485Topic;
                slaveId = _slaveId;
                name = _name;
                target_timestamp = _target_timestamp;
            }

            virtual void rs485Callback(const sonia_common::SendRS485Msg &receivedData)
            {
                if(receivedData.slave == slaveId)
                {
                    timestamp = CommonHardware::getCurrentTimeMs();
                }
            }

            virtual void rs485Publish()
            {
                sonia_common::SendRS485Msg msg;
                msg.slave = slaveId;

                msg.cmd = sonia_common::SendRS485Msg::CMD_KEEP_ALIVE;

                rs485Topic->publish(msg);
            }

            virtual bool detection()
            {
                return CommonSoftware::timeDetectionAlgorithm(timestamp, target_timestamp);
            }

            virtual bool correction()
            {
                return true;
            }

            virtual std::string getName()
            {
                return name;
            }

        private:
            unsigned int target_timestamp;
            uint8_t slaveId;
            ros::Publisher* rs485Topic;

            std::string name;
            std::chrono::milliseconds timestamp;
    };

    class BoardPowerSupply : public Rs485KeepAliveBoard
    {
        public:
            BoardPowerSupply(ros::Publisher* _rs485Topic, uint8_t _slaveId, std::string _name, unsigned int _target_timestamp):
            Rs485KeepAliveBoard(_rs485Topic, _slaveId, _name, _target_timestamp)
            {
            }
    };

    class BoardKillMission : public Rs485KeepAliveBoard
    {
        public:
            BoardKillMission(ros::Publisher* _rs485Topic, uint8_t _slaveId, std::string _name, unsigned int _target_timestamp):
            Rs485KeepAliveBoard( _rs485Topic, _slaveId, _name, _target_timestamp)
            {
            }
    };

    class BoardIo : public Rs485KeepAliveBoard
    {
        public:
            BoardIo(ros::Publisher* _rs485Topic, uint8_t _slaveId, std::string _name, unsigned int _target_timestamp):
            Rs485KeepAliveBoard( _rs485Topic, _slaveId, _name, _target_timestamp)
            {
            }
    };

    class BoardEsc : public Rs485KeepAliveBoard
    {
        public:
            BoardEsc(ros::Publisher* _rs485Topic, uint8_t _slaveId, std::string _name, unsigned int _target_timestamp):
            Rs485KeepAliveBoard( _rs485Topic, _slaveId, _name, _target_timestamp)
            {
            }
    };
}

#endif