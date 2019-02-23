#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <robot_and_sensor_hw/robot_and_sensor_hw.h>

int main ( int argc, char** argv )
{
    ros::init ( argc, argv, "DummyApp" );

    ros::AsyncSpinner spinner ( 1 );
    spinner.start();

    ros::NodeHandle nh;
    robot_and_sensor_hw::RobotAndSensorHW hw;
    bool init_success = hw.init ( nh, nh );

    if ( !init_success ) {
        ROS_ERROR("Could not initialize RobotAndSensorHW!");
        ros::shutdown();
    }

    controller_manager::ControllerManager cm ( &hw, nh );

    double frequency;
    nh.param ( "Node/frequency", frequency, 200. );
    ros::Duration period ( 1./frequency );
    while ( ros::ok() ) {
        if ( init_success ) {}
        hw.read ( ros::Time::now(), period );
        cm.update ( ros::Time::now(), period );
        hw.write ( ros::Time::now(), period );
        period.sleep();
    }

    ros::waitForShutdown();
}
