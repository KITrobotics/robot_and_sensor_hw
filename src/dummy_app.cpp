///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2012, Shadow Robot Company Ltd.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of Shadow Robot Company Ltd. nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//////////////////////////////////////////////////////////////////////////////

#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
// #include <combined_robot_hw/combined_robot_hw.h>
// #include <hardware_interface/sensor_hw.h>
#include <robot_and_sensor_hw/robot_and_sensor_hw.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "DummyApp");

  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::NodeHandle nh;
  //   combined_robot_hw::CombinedRobotHW hw;
  robot_and_sensor_hw::RobotAndSensorHW hw;
  bool init_success = hw.init(nh, nh);

  controller_manager::ControllerManager cm(&hw, nh);
  
//   pluginlib::ClassLoader<hardware_interface::SensorHW> poly_loader("hardware_interface", "hardware_interface::SensorHW");
//   try
//   {
//     boost::shared_ptr<hardware_interface::SensorHW> triangle = poly_loader.createInstance("testing/MySensorHW");
//       triangle->init(nh, nh);
// //       ROS_INFO("Triangle area");
//   }
//   catch(pluginlib::PluginlibException& ex)
//   {
//     ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
//   }

  ros::Duration period(1./200.);
  while (ros::ok())
  {
    if (init_success) {}
//     ROS_INFO("loop");
    hw.read(ros::Time::now(), period);
    cm.update(ros::Time::now(), period);
    hw.write(ros::Time::now(), period);
    period.sleep();
  }
  
  ros::waitForShutdown();
}

