#include "ros/ros.h"
#include <ralp_msgs/input_device.h>
#include <ralp_msgs/paths.h>

#include "ralp_msgs/teensy_input.h"
/*
 * VARIABLES
 */

float fX_delta = 0.0;        // delta x
float fY_delta = 0.0;        // delta y
float fButton  = 0.0;          // stylus button
int   iButton  = 0;

/**
 * This callback is to get the input_device message
 */
void tablet_callback(const ralp_msgs::input_device::ConstPtr& msg)
{
  fX_delta = msg->delta.x;
  fY_delta = msg->delta.y;
  iButton  = (int)msg->buttons; //msg->state.state;

  if (iButton == 65 || iButton == 129)
     fButton = 1.0;
  else
	   fButton = 0.0;
}

/*
 * MAIN
 */
int main(int argc, char** argv)
{
  // Initialize ROS
	ros::init(argc, argv, "teensy_serial");

	// Create a Node Handle
	ros::NodeHandle n;

  // Create ROS subscriber
  //ros::Subscriber sub = n.subscribe("/ralp/input_device", 1000, calm_udp_callback);
  ros::Subscriber sub = n.subscribe<ralp_msgs::input_device>(INPUT_TOPIC, 1000, tablet_callback);

  ros::Publisher tablet_pub = n.advertise<ralp_msgs::teensy_input>("/ralp_msgs/teensy_input", 10);

	// Set the publishing rate
	ros::Rate loop_rate(100); // publishing at 100Hz

  while(ros::ok())
  {
    ralp_msgs::teensy_input msgt;

    msgt.buttons = iButton;
    msgt.deltax  = fX_delta;
    msgt.deltay  = fY_delta;

    //msg.data = iButton;
    std::cout << "B: "      << iButton
              << "    dX: " << fX_delta
              << "    dY: " << fY_delta << std::endl;

    tablet_pub.publish(msgt);

    // ros::spin() will enter a loop
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
