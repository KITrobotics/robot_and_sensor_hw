
#ifndef ROBOT_AND_SENSOR_HW_ROBOT_AND_SENSOR_HW_H
#define ROBOT_AND_SENSOR_HW_ROBOT_AND_SENSOR_HW_H

#include <list>
#include <map>
#include <typeinfo>
#include <hardware_interface/internal/demangle_symbol.h>
#include <hardware_interface/internal/interface_manager.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/sensor_hw.h>
#include <pluginlib/class_loader.hpp>
#include <ros/console.h>
#include <ros/node_handle.h>

namespace robot_and_sensor_hw
{
    class RobotAndSensorHW : public hardware_interface::RobotHW
    {
    public:
        RobotAndSensorHW();
        
        virtual ~RobotAndSensorHW(){}
        virtual bool init(ros::NodeHandle& root_nh, ros::NodeHandle &robot_hw_nh);
          
          
          /**
           * Check (in non-realtime) if given controllers could be started and stopped from the current state of the RobotHW
           * with regard to necessary hardware interface switches and prepare the switching. Start and stop list are disjoint.
           * This handles the check and preparation, the actual switch is commited in doSwitch()
           */
        virtual bool prepareSwitch(const std::list<hardware_interface::ControllerInfo>& start_list,
                                     const std::list<hardware_interface::ControllerInfo>& stop_list);
          
          /**
           * Perform (in realtime) all necessary hardware interface switches in order to start and stop the given controllers.
           * Start and stop list are disjoint. The feasability was checked in prepareSwitch() beforehand.
           */
        virtual void doSwitch(const std::list<hardware_interface::ControllerInfo>& start_list,
                            const std::list<hardware_interface::ControllerInfo>& stop_list);

        /**
        * Reads data from the robot and sensor HW
        *
        * \param time The current time
        * \param period The time passed since the last call to \ref read
        */
        virtual void read(const ros::Time& time, const ros::Duration& period);

        /**
        * Writes data to the robot HW
        *
        * \param time The current time
        * \param period The time passed since the last call to \ref write
        */
        virtual void write(const ros::Time& time, const ros::Duration& period);
          
    protected:
        ros::NodeHandle root_nh_;
        ros::NodeHandle robot_hw_nh_;
        pluginlib::ClassLoader<hardware_interface::RobotHW> robot_hw_loader_;
        pluginlib::ClassLoader<hardware_interface::SensorHW> sensor_hw_loader_;
        hardware_interface::RobotHWSharedPtr robot_hw_;
        hardware_interface::SensorHWSharedPtr sensor_hw_;
        
        virtual bool loadHW(const std::string& name, bool isSensor = false);
    };
    
}

#endif
