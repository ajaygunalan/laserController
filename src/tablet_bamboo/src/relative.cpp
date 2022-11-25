/**
 * @file	relative.cpp
 * @author	Emidio Olivieri
 * @version	1.0
 * @date	16-Dec-2015
 * @brief	Tablet input device to be used when pen-touched and in relative mode
 * @details	This program replace touchtablet and it is used to enable Wacom Bamboo
 *          graphic tablets as input device and properly connect it to ROS. Messages
 *          are sent only when touching the tablet with the pen and they will
 *          correspond to the distance to the first point touched with respect to
 *          the first point touched
 */

/*
 * Preliminar notes:
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
 *
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
//#define MODE_JOYSTICK //Keep velocity value
//#define ENABLE_WATCHDOG

#define DEFAULT_SCALE (1.0) //Value corresponding to the default scale of the movement
#define SCALE_FACTOR (1.1)
#define MAX_SCALE (4.177)
#define MIN_SCALE (0.38555)

#ifdef MODE_JOYSTICK
#define MULT_PAR_X (4.0) //Value corresponding to the default scale of the movement
#define MULT_PAR_Y (6.0) //Value corresponding to the default scale of the movement
#else
#define MULT_PAR_X (400.0) //Value corresponding to the default scale of the movement
#define MULT_PAR_Y (600.0) //Value corresponding to the default scale of the movement
#endif

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
#include "tablet_bamboo/RALP_bambooHandler.hpp"
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


int main(int argc, char** argv) {
    RALP_bambooHandler* handler = RALP_bambooHandler::firstDevice();
    if(handler == 0)
    {
    	ROS_ERROR("No device found or connected!");
        return -1;
    }

    ros::init(argc, argv, "RelativeBambooInputDevice");
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

    double scale;
    if (node.hasParam(INPUT_TABLET_SCALE_PARAM) &&
        node.getParam(INPUT_TABLET_SCALE_PARAM, scale)){
        ROS_INFO("Set scale to %f", scale);
    } else {
        ROS_INFO("No scale set on parameter %s: set by default to %f", INPUT_TABLET_SCALE_PARAM, DEFAULT_SCALE);
        scale = DEFAULT_SCALE;
    }

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

    double firstX = 0.0, firstY = 0.0, lastX = 0.0, lastY = 0.0, x, y;
    ralp_msgs::input_device inputDeviceMsg;
    inputDeviceMsg.pose.orientation.w = 0.0;
    inputDeviceMsg.pose.orientation.x = 0.0;
    inputDeviceMsg.pose.orientation.y = 0.0;
    inputDeviceMsg.pose.orientation.z = 0.0;
    inputDeviceMsg.pose.position.x = 0.5;
    inputDeviceMsg.pose.position.y = 0.5;
    inputDeviceMsg.pose.position.z = 0.0;
    inputDeviceMsg.delta.x = 0.0;
    inputDeviceMsg.delta.y = 0.0;
    inputDeviceMsg.delta.z = 0.0;
    inputDeviceMsg.buttons = (unsigned char)0;
    inputDeviceMsg.pressure = 0.0;
    inputDeviceMsg.state.state = 0;
    inputDeviceMsg.header.stamp = ros::Time::now();
    {
        struct timespec tim;
        tim.tv_sec = 0l;
        tim.tv_nsec = 1000000l; //1 ms
        ROS_INFO("Waiting for subscriber");
        while(pub.getNumSubscribers() < 1) {
            nanosleep(&tim,0);
        }
        ROS_INFO("Done. Central position set");
    }

    pub.publish(inputDeviceMsg);


    bool fForw, forw, fBack, back, fRight, right, fLeft, left, pressed,
            lastPressed = false, st1, st2;
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
        if(!handler->getNormPos(&x, &y, &inputDeviceMsg.delta.z, &inputDeviceMsg.pressure))
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

            if(forw && !fForw){
                scale*=SCALE_FACTOR;
                if(scale > MAX_SCALE)
                    scale = MAX_SCALE;
                else
                    ROS_INFO("Scale changed to %f", scale);
            }
            fForw = forw;
            if(back && !fBack){
                scale/=SCALE_FACTOR;
                if(scale < MIN_SCALE)
                    scale = MIN_SCALE;
                else
                    ROS_INFO("Scale changed to %f", scale);
            }
            fBack = back;
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

        if(pressed) {
            if(lastPressed){
                inputDeviceMsg.pose.position.x += scale*(x-lastX);
//                if(inputDeviceMsg.pose.position.x < 0.0)
//                    inputDeviceMsg.pose.position.x = 0.0;
//                else if(inputDeviceMsg.pose.position.x > 1.0)
//                    inputDeviceMsg.pose.position.x = 1.0;
#ifdef MODE_JOYSTICK
                inputDeviceMsg.delta.x = scale * (x-firstX) * MULT_PAR_X;
#else
                inputDeviceMsg.delta.x = scale * (x-lastX) * MULT_PAR_X;
#endif
                lastX = x;

                inputDeviceMsg.pose.position.y += scale*(y-lastY);
//                if(inputDeviceMsg.pose.position.y < 0.0)
//                    inputDeviceMsg.pose.position.y = 0.0;
//                else if(inputDeviceMsg.pose.position.y > 1.0)
//                    inputDeviceMsg.pose.position.y = 1.0;
#ifdef MODE_JOYSTICK
                inputDeviceMsg.delta.y = scale * (y-firstY) * MULT_PAR_Y;
#else
                inputDeviceMsg.delta.y = scale * (y-lastY) * MULT_PAR_Y;
#endif
                lastY = y;
            } else {
                lastPressed = true;
                lastX = x;
                lastY = y;

                inputDeviceMsg.delta.x = 0.0;
                inputDeviceMsg.delta.y = 0.0;
                firstX = x;
                firstY = y;
            }

        } else {
            inputDeviceMsg.delta.x = 0.0;
            inputDeviceMsg.delta.y = 0.0;

            // firstX = x;
            // firstY = y;

            lastPressed = false;
        }

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
