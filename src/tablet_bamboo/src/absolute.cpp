/**
 * @file	absolute.cpp
 * @author	Emidio Olivieri
 * @version	1.0
 * @date	16-Dec-2015
 * @brief	Tablet input device to be used when pen-touched and in absolute mode
 * @details	This program replace tablet and it is used to enable Wacom Bamboo
 *          graphic tablets as input device and properly connect it to ROS. Messages
 *          are sent only when touching the tablet with the pen and they will
 *          correspond to the absolute point touched
 */

/*
 * Preliminary notes:
 *
 * BUTTONS:
 * bit1 (0x01) -> pentouch
 * bit2 (0x02) -> fingertouch (not used, always 0)
 * bit3 (0x04) -> leftbutton
 * bit4 (0x08) -> rightbutton
 * bit5 (0x10) -> "forward"button
 * bit6 (0x20) -> "back"button
 * bit7 (0x40) -> stylusbutton
 * bit8 (0x80) -> stylusbutton2
 * OxOO if not pressing, OxFF if pressing
 * STATUS:
 * Activate and Record -> stylusbutton (1 or 2) pressed with pentouch active
 * Cancel -> touch without pressing
 * PRESSURE/Z:
 * unused (0.0)
 */

/*
 * DEFINES
 */
    /* Functional parameters */
//#define ENABLE_WATCHDOG

    /* Buttons */
#define PENT 0x01
#define FINGERT 0x02
#define LEFTB 0x04
#define RIGHTB 0x08
#define FORWARDB 0x10
#define BACKB 0x20
#define STYLUSB 0x40
#define STYLUS2B 0x80

#ifdef ENABLE_WATCHDOG
#define UDP_PORT		23000
#define REMOTE_ADDR		"169.254.89.51"
#endif

/*
 * INCLUDES
 */
    /* General */
#include <tablet_bamboo/RALP_bambooHandler.hpp>
    /* ROS */
// General
#include <ros/ros.h>
// Messages
#include <ralp_msgs/input_device.h>
#include <ralp_msgs/input_state.h>
#include <ralp_msgs/paths.h>
    /* Watchdog */
#ifdef ENABLE_WATCHDOG
#include "watchdog.h"
#endif


int main(int argc, char** argv)
{
	RALP_bambooHandler* handler = RALP_bambooHandler::firstDevice();
    if(handler == 0)
    {
    	ROS_ERROR("No device found or connected!");
        return -1;
    }

    ros::init(argc, argv, "AbsoluteBambooInputDevice");
    ros::NodeHandle node;
    ros::Rate loop_rate(100);

    RALP_bambooHandler::Rotation rotation;
    int rotateModality;
    if (node.hasParam(INPUT_TABLET_ROTATE_PARAM) &&
        node.getParam(INPUT_TABLET_ROTATE_PARAM, rotateModality)){
        switch(rotateModality){
        case 0:
            ROS_INFO("Straight modality set");
            rotation = RALP_bambooHandler::STRAIGHT;
            break;
        case 90:
            ROS_INFO("Rotate90 modality set");
            rotation = RALP_bambooHandler::ROT90DEG;
            break;
        case 180:
            ROS_INFO("Rotate180 modality set");
            rotation = RALP_bambooHandler::ROT180DEG;
            break;
        case 270:
            ROS_INFO("Rotate270 modality set");
            rotation = RALP_bambooHandler::ROT270DEG;
            break;
        default:
            ROS_ERROR("Passed parameter is invalid");
            ROS_INFO("Set by default to straight");
            rotation = RALP_bambooHandler::STRAIGHT;
            break;
        }
    } else {
        ROS_INFO("No rotation set on parameter %s: set by default to straight", INPUT_TABLET_ROTATE_PARAM);
        rotation = RALP_bambooHandler::STRAIGHT;
    }
    handler->setRotate(rotation);

    bool buttonUsed;
    if (node.hasParam(INPUT_TABLET_BUTTON_PARAM)) {
        ROS_INFO("Button usable");
        buttonUsed = true;
    } else {
        ROS_INFO("No parameter %s: buttons will be disabled", INPUT_TABLET_BUTTON_PARAM);
        buttonUsed = false;
    }

    ros::Publisher pub=
            node.advertise<ralp_msgs::input_device>(INPUT_TOPIC, 1);

    ros::start();

    ralp_msgs::input_device inputDeviceMsg;
    inputDeviceMsg.pose.orientation.w = 0.0;
    inputDeviceMsg.pose.orientation.x = 0.0;
    inputDeviceMsg.pose.orientation.y = 0.0;
    inputDeviceMsg.pose.orientation.z = 0.0;
    inputDeviceMsg.pose.position.x = 0.0;
    inputDeviceMsg.pose.position.y = 0.0;
    inputDeviceMsg.pose.position.z = 0.0;
    inputDeviceMsg.delta.x = 0.0;
    inputDeviceMsg.delta.y = 0.0;
    inputDeviceMsg.delta.z = 0.0;
    inputDeviceMsg.buttons = (unsigned char)0;
    inputDeviceMsg.pressure = 0.0;
    inputDeviceMsg.state.state = 0;


    bool fForw, forw, fBack, back, fRight, right, fLeft, left, pressed, st1, st2;
    handler->getButtonState(&fLeft, &fRight, &fForw, &fBack);

#ifdef ENABLE_WATCHDOG
		/* Init Watchdog */

	Watchdog watchdog;
	watchdog.configure(REMOTE_ADDR, UDP_PORT, 250, 1);
	unsigned long watchdogMask;
	watchdogMask = watchdog.getWatcherMask("input");
	int dividerMax = 15;
	int divider = dividerMax;
#endif


    while(ros::ok()) {
        handler->getButtonState(&left, &right, &forw, &back, &st1, &st2);
        if(!handler->getNormPos(&inputDeviceMsg.pose.position.x,
                                &inputDeviceMsg.pose.position.y,
                                &inputDeviceMsg.delta.z, &inputDeviceMsg.pressure))
            continue;
        pressed = handler->isTouching();
        if(buttonUsed) {
            if(right && !fRight) {
                switch(rotation){
                case RALP_bambooHandler::STRAIGHT:
                    ROS_INFO("Rotate270 modality set");
                    rotation = RALP_bambooHandler::ROT270DEG;
                    break;
                case RALP_bambooHandler::ROT90DEG:
                    ROS_INFO("Straight modality set");
                    rotation = RALP_bambooHandler::STRAIGHT;
                    break;
                case RALP_bambooHandler::ROT180DEG:
                    ROS_INFO("Rotate90 modality set");
                    rotation = RALP_bambooHandler::ROT90DEG;
                    break;
                case  RALP_bambooHandler::ROT270DEG:
                    ROS_INFO("Rotate180 modality set");
                    rotation = RALP_bambooHandler::ROT180DEG;
                    break;
                default:
                    ROS_ERROR("Unknown modality: something wrong just happened");
                    ROS_INFO("Set by default to straight");
                    rotation = RALP_bambooHandler::STRAIGHT;
                }
                handler->setRotate(rotation);
            }
            fRight = right;

            if(left && !fLeft) {
                switch(rotation){
                case RALP_bambooHandler::STRAIGHT:
                    ROS_INFO("Rotate90 modality set");
                    rotation = RALP_bambooHandler::ROT90DEG;
                    break;
                case RALP_bambooHandler::ROT90DEG:
                    ROS_INFO("Rotate180 modality set");
                    rotation = RALP_bambooHandler::ROT180DEG;
                    break;
                case RALP_bambooHandler::ROT180DEG:
                    ROS_INFO("Rotate270 modality set");
                    rotation = RALP_bambooHandler::ROT270DEG;
                    break;
                case  RALP_bambooHandler::ROT270DEG:
                    ROS_INFO("Straight modality set");
                    rotation = RALP_bambooHandler::STRAIGHT;
                    break;
                default:
                    ROS_ERROR("Unknown modality: something wrong just happened");
                    ROS_INFO("Set by default to straight");
                    rotation = RALP_bambooHandler::STRAIGHT;
                }
                handler->setRotate(rotation);
            }
            fLeft = left;
        }

        inputDeviceMsg.buttons =
                (pressed?PENT:0x00) |
                (left?LEFTB:0x00) |
                (right?RIGHTB:0x00) |
                (forw?FORWARDB:0x00) |
                (back?BACKB:0x00) |
                (st1?STYLUSB:0x00) |
                (st2?STYLUS2B:0x00);

        if(st2){
            inputDeviceMsg.state.state = ralp_msgs::input_state::CANCEL;
        } else if(st1) {
            inputDeviceMsg.state.state = ralp_msgs::input_state::ACTIVATE |
                    ralp_msgs::input_state::RECORD;;
        } else
            inputDeviceMsg.state.state = 0;

        inputDeviceMsg.header.stamp = ros::Time::now();
        pub.publish(inputDeviceMsg);
        loop_rate.sleep();

#ifdef ENABLE_WATCHDOG
        divider--;
        if (divider <= 0)
        {
        	watchdog.ready(watchdogMask);
        	divider = dividerMax;
        }
#endif
    }

#ifdef ENABLE_WATCHDOG
    //watchdog.disconnect();
#endif
    return 0;
}
