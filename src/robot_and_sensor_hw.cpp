
#include <algorithm>
#include "robot_and_sensor_hw/robot_and_sensor_hw.h"

namespace robot_and_sensor_hw
{
RobotAndSensorHW::RobotAndSensorHW() :
    robot_hw_loader_ ( "hardware_interface", "hardware_interface::RobotHW" ),
    sensor_hw_loader_ ( "hardware_interface", "hardware_interface::SensorHW" )
{}

bool RobotAndSensorHW::init ( ros::NodeHandle& root_nh, ros::NodeHandle &robot_hw_nh )
{
    root_nh_ = root_nh;
    robot_hw_nh_ = robot_hw_nh;

    std::string robot;
    std::string robot_param_name = "robot_hardware";
    if ( !robot_hw_nh.getParam ( robot_param_name, robot ) ) {
        ROS_ERROR_STREAM ( "Could not find '" << robot_param_name << "' parameter (namespace: " << robot_hw_nh.getNamespace() << ")." );
        return false;
    }

    std::string sensor;
    std::string sensor_param_name = "sensor_hardware";
    if ( !robot_hw_nh.getParam ( sensor_param_name, sensor ) ) {
        ROS_ERROR_STREAM ( "Could not find '" << sensor_param_name << "' parameter (namespace: " << robot_hw_nh.getNamespace() << ")." );
        return false;
    }


    if ( !loadHW ( robot ) || !loadHW ( sensor, true ) ) {
        return false;
    }

    return true;
}

bool RobotAndSensorHW::prepareSwitch ( const std::list<hardware_interface::ControllerInfo>& start_list,
                                       const std::list<hardware_interface::ControllerInfo>& stop_list )
{
    return robot_hw_->prepareSwitch ( start_list, stop_list );
}

void RobotAndSensorHW::doSwitch ( const std::list<hardware_interface::ControllerInfo>& start_list,
                                  const std::list<hardware_interface::ControllerInfo>& stop_list )
{
    robot_hw_->doSwitch ( start_list, stop_list );
}

bool RobotAndSensorHW::loadHW ( const std::string& name, bool isSensor )
{
    std::string hw_type = "robot";
    if ( isSensor ) {
        hw_type = "sensor";
    }
    ROS_DEBUG ( "Will load %s HW '%s'", hw_type.c_str(), name.c_str() );

    ros::NodeHandle c_nh;
    try {
        c_nh = ros::NodeHandle ( robot_hw_nh_, name );
    } catch ( std::exception &e ) {
        ROS_ERROR ( "Exception thrown while constructing nodehandle for %s HW with name '%s':\n%s", hw_type.c_str(), name.c_str(), e.what() );
        return false;
    } catch ( ... ) {
        ROS_ERROR ( "Exception thrown while constructing nodehandle for %s HW with name '%s'", hw_type.c_str(), name.c_str() );
        return false;
    }

    std::string type;
    if ( c_nh.getParam ( "type", type ) ) {
        ROS_DEBUG ( "Constructing %s HW '%s' of type '%s'", hw_type.c_str(), name.c_str(), type.c_str() );
        try {
            std::vector<std::string> cur_types;
            if ( isSensor ) {
                cur_types = sensor_hw_loader_.getDeclaredClasses();
            } else {
                cur_types = robot_hw_loader_.getDeclaredClasses();
            }
            for ( size_t i=0; i < cur_types.size(); i++ ) {
                if ( type == cur_types[i] ) {
                    if ( isSensor ) {
                        sensor_hw_ = sensor_hw_loader_.createInstance ( type );
                    } else {
                        robot_hw_ = robot_hw_loader_.createInstance ( type );
                    }
                }
            }
        } catch ( const std::runtime_error &ex ) {
            ROS_ERROR ( "Could not load class %s: %s", type.c_str(), ex.what() );
        }
    } else {
        ROS_ERROR ( "Could not load %s HW '%s' because the type was not specified. Did you load the %s HW configuration on the parameter server (namespace: '%s')?", hw_type.c_str(), name.c_str(), hw_type.c_str(), c_nh.getNamespace().c_str() );
        return false;
    }

    bool created = true;
    if ( isSensor ) {
        if ( !sensor_hw_ ) {
            created = false;
        }
    } else {
        if ( !robot_hw_ ) {
            created = false;
        }
    }
    if ( !created ) {
        ROS_ERROR ( "Could not load %s HW '%s' because %s HW type '%s' does not exist.", hw_type.c_str(), name.c_str(),         hw_type.c_str(), type.c_str() );
        return false;
    }

    ROS_DEBUG ( "Initializing %s HW '%s'", hw_type.c_str(), name.c_str() );
    bool initialized = false;
    try {
        if ( isSensor ) {
            initialized = sensor_hw_->init ( root_nh_, c_nh );
        } else {
            initialized = robot_hw_->init ( root_nh_, c_nh );
        }
    } catch ( std::exception &e ) {
        ROS_ERROR ( "Exception thrown while initializing %s HW %s.\n%s", hw_type.c_str(), name.c_str(), e.what() );
        initialized = false;
    } catch ( ... ) {
        ROS_ERROR ( "Exception thrown while initializing %s HW %s", hw_type.c_str(), name.c_str() );
        initialized = false;
    }

    if ( !initialized ) {
        ROS_ERROR ( "Initializing %s HW '%s' failed", hw_type.c_str(), name.c_str() );
        return false;
    }
    ROS_DEBUG ( "Initialized %s HW '%s' successful", hw_type.c_str(), name.c_str() );

    if ( isSensor ) {
        this->registerInterfaceManager ( sensor_hw_.get() );
    } else {
        this->registerInterfaceManager ( robot_hw_.get() );
    }

    ROS_DEBUG ( "Successfully load %s HW '%s'", hw_type.c_str(), name.c_str() );
    return true;
}

void RobotAndSensorHW::read ( const ros::Time& time, const ros::Duration& period )
{
    robot_hw_->read ( time, period );
    sensor_hw_->read ( time, period );
}


void RobotAndSensorHW::write ( const ros::Time& time, const ros::Duration& period )
{
    robot_hw_->write ( time, period );
}
}



