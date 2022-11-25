#include "tablet_bamboo/bambooHandler.hpp"
#include <sys/ioctl.h>
#include <fcntl.h>
#include <dirent.h>
#include <cstring>
#include <cstdlib>
#include <cstdio>
#include <unistd.h>
#include <errno.h>
#include <iostream>

#include <linux/input.h>
#include <X11/extensions/XInput.h>
#include <X11/Xlib.h>
#include <X11/Xatom.h>

#define DEV_INPUT_EVENT "/dev/input/"
#define EVENT_DEV_NAME "event"
#define COMMON_STRING "Wacom" // before: "Wacom Bamboo" -> allow Wacom Intuos to be detected as well
#define PEN_STRING "Pen"
#define FINGER_STRING "Finger"

#define CALLBACK_NS 500000l //0.5 ms

//Next defines are where the values are stored in data array
#define ABS_X_POS 0
#define ABS_Y_POS 1
#define ABS_PRESSURE_POS 2
#define ABS_DISTANCE_POS 3
//Next defines are where the values are stored in buttons array
#define BTN_TOUCH_POS 0
#define BTN_STYLUS_POS 1
#define BTN_STYLUS2_POS 2
#define BTN_LEFT_POS 3
#define BTN_RIGHT_POS 4
#define BTN_FORWARD_POS 5
#define BTN_BACK_POS 6

#define NUM_EV_ONCE 64 //Maximum number of events handled at once

//#define DEBUG_DATA_READ //if defined it outputs data read values
//#define DEBUG_BUTTON_READ //if defined it outputs button read values

BambooHandler::BambooHandler(bool pen, bool disable) :
    pen(pen), disable(disable), initialized(false),
    dpy(0), deviceList(0) {
    for(unsigned char i = 0; i < 4; i++)
        data[i] = new input_absinfo;
    pthread_mutex_init(&dataMutex, 0);
    pthread_mutex_init(&threadMutex, 0);
}

BambooHandler::~BambooHandler() {
    if(isInitialized())
        deInit();
    for(unsigned char i = 0; i < 4; i++)
        delete data[i];
}

bool BambooHandler::init() {
    struct dirent **nameList;
    int i, ndev, dev, j = 1;

    ndev = scandir(DEV_INPUT_EVENT, &nameList, eventFilter, versionsort);
    if (ndev <= 0) {
        std::cerr << "Error in seeking devices..." << std::endl;
        return false;
    }

    std::vector<int> events;
    events.resize(1);
    events[0] = -1;
    std::string path(DEV_INPUT_EVENT);
    char* eventName, name[256];

    for(i = 0; i < ndev; i++) {
        eventName = nameList[i]->d_name;
        //std::cout << eventName;
        dev = open((path + eventName).c_str(), O_RDONLY|O_NONBLOCK);
        if(dev >= 0) {
            ioctl(dev, EVIOCGNAME(sizeof(name)), name);
            //std::cout << "eventName = " << name;

            if(strstr(name,COMMON_STRING)!=0){
                //it is a bamboo event
                if(pen && strstr(name,PEN_STRING)!=0 ||
                        !pen && strstr(name,FINGER_STRING)!=0){
                    events[0] = extractNumberFromEvent(eventName);
                    std::cout << "Wacom core device found: " << eventName
                              << " (" << name << ")" << std::endl;
                } else {
                    events.resize(j+1);
                    events[j++] = extractNumberFromEvent(eventName);
                    std::cout << "Wacom device found: " << eventName
                              << " (" << name << ")" << std::endl;
                }
            }
            close(dev);
        }

        delete nameList[i];
    }
    delete[] nameList;

    if(events[0] < 0)
        return false;
    else
        return init(events);
}

bool BambooHandler::init(std::vector<int> events) {
    if(disable) { //disable
        Display* dpy = XOpenDisplay(NULL);
        if(dpy==0) {
            std::cerr << "XOpenDisplay failed\n";
            return false;
        } else {
            this->dpy = dpy;
        }

        devEn=XInternAtom(dpy,"Device Enabled",False);
        unsigned char flag = 0;

        XDeviceInfo* deviceList;
        deviceList = XListInputDevices(dpy, &devNum);
        this->deviceList = deviceList;

        XDevice* dev;
        for(int i=0; i<devNum; i++) {
            if(strstr(deviceList[i].name, COMMON_STRING) != 0) {
                dev = XOpenDevice(dpy, deviceList[i].id);
                if (dev == 0) {
                    std::cerr << "Unable to access device " << deviceList[i].name << std::endl;
                    return false;
                }

                XChangeDeviceProperty(dpy, dev, devEn, XA_INTEGER, 8,
                                      PropModeReplace, &flag, 1);

                std::cout << "Acquiring " << deviceList[i].name << " from X server into ROS!" << std::endl;
                XFlush(dpy);
                XCloseDevice(dpy, dev);
            }
        }
    }

    char buffer[50];

    devices.resize(events.size());
    for(int i = 0; i < events.size(); i++) {
        sprintf(buffer, "%s%s%d", DEV_INPUT_EVENT, "event", events[i]);
        devices[i] = open(buffer, O_RDONLY|O_NONBLOCK);
        if(devices[i] < 0) { //error
            std::cerr << "Impossible to re-open " << buffer << std::endl;
            for(int j = 0; j < i; j++)
                close(devices[i]);
            devices.resize(0);
            initialized = true;
            deInit();
            return false;
        }
    }

    if(pthread_create(&readThreadH, 0, staticReadThread, (void*)this) != 0) {
        initialized = true;
        deInit();
        return false;
    } else {
        initialized = true;
        return true;
    }
}

void BambooHandler::deInit() {
    if(!isInitialized())
        return;

    pthread_mutex_lock(&threadMutex);
    if(threadRunning) {
        threadRunning = false;
        pthread_mutex_unlock(&threadMutex);
        struct timespec tim;
        tim.tv_sec = 0l;
        tim.tv_nsec = 1000000l; //1 ms

        pthread_mutex_lock(&threadMutex);
        while(!threadRunning){
            pthread_mutex_unlock(&threadMutex); //this must be changed by the other one
            nanosleep(&tim,0);
            pthread_mutex_lock(&threadMutex);
        }
        threadRunning = false;
    }
    pthread_mutex_unlock(&threadMutex);

    for(unsigned char i = 0; i < devices.size(); i++) {
        close(devices[i]);
    }

    if(disable) { //reenable
        Display* dpy = (Display*) this->dpy;

        unsigned char flag = 1;

        XDeviceInfo* deviceList = (XDeviceInfo*) this->deviceList;
        XDevice* dev;

        for(int i=0; i<devNum; i++) {
            if(strstr(deviceList[i].name, COMMON_STRING) != 0) {
                dev = XOpenDevice(dpy, deviceList[i].id);
                if (dev != 0) {
                    XChangeDeviceProperty(dpy, dev, devEn, XA_INTEGER, 8,
                                          PropModeReplace, &flag, 1);
                    std::cout << "Releasing " << deviceList[i].name << " from ROS into X server!" << std::endl;

                    XFlush(dpy);
                    XCloseDevice(dpy, dev);
                }
            }
        }
        delete[] deviceList;
        this->deviceList = 0;
        XCloseDisplay(dpy);
        this->dpy = 0;
    }

    initialized = false;
}

bool BambooHandler::getPosition(int *xVal, int *yVal, int *xMax, int *yMax,
                                int *xMin, int *yMin, int *xRes, int *yRes) {
    if(!isInitialized())
        return false;
    pthread_mutex_lock(&dataMutex);
    if(xVal != 0)
        *xVal = data[ABS_X_POS]->value;
    if(yVal != 0)
        *yVal = data[ABS_Y_POS]->value;
    if(xMax != 0)
        *xMax = data[ABS_X_POS]->maximum;
    if(yMax != 0)
        *yMax = data[ABS_Y_POS]->maximum;
    if(xMin != 0)
        *xMin = data[ABS_X_POS]->minimum;
    if(yMin != 0)
        *yMin = data[ABS_Y_POS]->minimum;
    if(xRes != 0)
        *xRes = data[ABS_X_POS]->resolution;
    if(yRes != 0)
        *yRes = data[ABS_Y_POS]->resolution;
    pthread_mutex_unlock(&dataMutex);
    return true;
}

bool BambooHandler::getDistance(int *distVal, int *distMax, int *distMin, int *distRes) {
    if(!isInitialized())
        return false;
    pthread_mutex_lock(&dataMutex);
    if(distVal != 0)
        *distVal = data[ABS_DISTANCE_POS]->value;
    if(distMax != 0)
        *distMax = data[ABS_DISTANCE_POS]->maximum;
    if(distMin != 0)
        *distMin = data[ABS_DISTANCE_POS]->minimum;
    if(distRes != 0)
        *distRes = data[ABS_DISTANCE_POS]->resolution;
    pthread_mutex_unlock(&dataMutex);
    return true;
}

bool BambooHandler::getPressure(int *pressVal, int *pressMax, int *pressMin, int *pressRes) {
    if(!isInitialized())
        return false;
    pthread_mutex_lock(&dataMutex);
    if(pressVal != 0)
        *pressVal = data[ABS_PRESSURE_POS]->value;
    if(pressMax != 0)
        *pressMax = data[ABS_PRESSURE_POS]->maximum;
    if(pressMin != 0)
        *pressMin = data[ABS_PRESSURE_POS]->minimum;
    if(pressRes != 0)
        *pressRes = data[ABS_PRESSURE_POS]->resolution;
    pthread_mutex_unlock(&dataMutex);
    return true;
}

bool BambooHandler::isTouching() {
    if(!isInitialized())
        return false;
    bool touching;
    pthread_mutex_lock(&dataMutex);
    touching = buttons[BTN_TOUCH_POS];
    pthread_mutex_unlock(&dataMutex);
    return touching;
}

bool BambooHandler::getButtonState(bool *left, bool *right, bool *forward,
                                   bool *backward, bool *stylus1, bool *stylus2) {
    if(!isInitialized())
        return false;
    pthread_mutex_lock(&dataMutex);
    if(left != 0)
        *left = buttons[BTN_LEFT_POS];
    if(right != 0)
        *right = buttons[BTN_RIGHT_POS];
    if(forward != 0)
        *forward = buttons[BTN_FORWARD_POS];
    if(backward != 0)
        *backward = buttons[BTN_BACK_POS];
    if(stylus1 != 0)
        *stylus1 = buttons[BTN_STYLUS_POS];
    if(stylus2 != 0)
        *stylus2 = buttons[BTN_STYLUS2_POS];
    pthread_mutex_unlock(&dataMutex);
    return true;
}

bool BambooHandler::isInitialized() {
    return initialized;
}

void BambooHandler::readThread() {
    pthread_mutex_lock(&threadMutex);
    threadRunning = true;
    struct timespec tim;
    tim.tv_sec = 0l;
    tim.tv_nsec = CALLBACK_NS;

    input_absinfo x, y;

    ioctl(devices[0], EVIOCGABS(ABS_X), &x);
    ioctl(devices[0], EVIOCGABS(ABS_Y), &y);

    while(threadRunning){
        pthread_mutex_unlock(&threadMutex);
        pthread_mutex_lock(&dataMutex);
        ioctl(devices[0], EVIOCGABS(ABS_X), data[ABS_X_POS]);
        ioctl(devices[0], EVIOCGABS(ABS_Y), data[ABS_Y_POS]);
        if(data[ABS_X_POS]->value != 0 || data[ABS_Y_POS]->value != 0) {
            x = *data[ABS_X_POS];
            y = *data[ABS_Y_POS];
        } else {
            *data[ABS_X_POS] = x;
            *data[ABS_Y_POS] = y;
        }
        ioctl(devices[0], EVIOCGABS(ABS_DISTANCE), data[ABS_DISTANCE_POS]);
        ioctl(devices[0], EVIOCGABS(ABS_PRESSURE), data[ABS_PRESSURE_POS]);

#ifdef DEBUG_DATA_READ
        system("clear");
        std::cout << "Positions: (" << data[ABS_X_POS]->value << ", " << data[ABS_Y_POS]->value
                  << ") in ("  << data[ABS_X_POS]->minimum << ", " << data[ABS_Y_POS]->minimum
                  << ")->("  << data[ABS_X_POS]->maximum << ", " << data[ABS_Y_POS]->maximum
                  << ") range with ("  << data[ABS_X_POS]->resolution << ", " << data[ABS_Y_POS]->resolution
                  << ") resolution" << std::endl;
        std::cout << "Distance: " << data[ABS_DISTANCE_POS]->value
                  << " in "  << data[ABS_DISTANCE_POS]->minimum << " -> "  << data[ABS_DISTANCE_POS]->maximum
                  << " range with " << data[ABS_DISTANCE_POS]->resolution << " resolution" << std::endl;
        std::cout << "Pressure: " << data[ABS_PRESSURE_POS]->value
                  << " in "  << data[ABS_PRESSURE_POS]->minimum << " -> "  << data[ABS_PRESSURE_POS]->maximum
                  << " range with " << data[ABS_PRESSURE_POS]->resolution << " resolution" << std::endl;
#endif

        struct input_event ev[NUM_EV_ONCE];
        int rd;
        bool val;

        for(unsigned char i = 0; i < devices.size(); i++) {
            //read button events on all devices
            rd = read(devices[i], ev, sizeof(struct input_event) * NUM_EV_ONCE);

            if(rd < 0) {
                //no new event available
                if(errno!=EAGAIN){
                    std::cerr << "Can't read from device" << std::endl;
                }
            } else if (rd % (int) sizeof(struct input_event)!=0) {
                std::cerr << "Unexpected number of bit read from device" << std::endl;
            } else {
                rd/=sizeof(struct input_event); //now rd is the number of event received
                for(int j = 0; j < rd; j++) {
                    if(ev[j].type == EV_KEY) {
                        val = (ev[j].value!=0);
                        switch(ev[j].code){
                        case BTN_TOUCH:
                            if(i == 0)
                                buttons[BTN_TOUCH_POS] = val;
#ifdef DEBUG_BUTTON_READ
                                if(val) std::cout << "BTN_TOUCH" << std::endl;
#endif
                            break;
                        case BTN_STYLUS:
#ifdef DEBUG_BUTTON_READ
                            if(val) std::cout << "BTN_STYLUS" << std::endl;
#endif
                            buttons[BTN_STYLUS_POS] = val;
                            break;
                        case BTN_STYLUS2:
#ifdef DEBUG_BUTTON_READ
                            if(val) std::cout << "BTN_STYLUS2" << std::endl;
#endif
                            buttons[BTN_STYLUS2_POS] = val;
                            break;
                        case BTN_LEFT:
#ifdef DEBUG_BUTTON_READ
                            if(val) std::cout << "BTN_LEFT" << std::endl;
#endif
                            buttons[BTN_LEFT_POS] = val;
                            break;
                        case BTN_RIGHT:
#ifdef DEBUG_BUTTON_READ
                            if(val) std::cout << "BTN_RIGHT" << std::endl;
#endif
                            buttons[BTN_RIGHT_POS] = val;
                            break;
                        case BTN_FORWARD:
#ifdef DEBUG_BUTTON_READ
                            if(val) std::cout << "BTN_FORWARD" << std::endl;
#endif
                            buttons[BTN_FORWARD_POS] = val;
                            break;
                        case BTN_BACK:
#ifdef DEBUG_BUTTON_READ
                            if(val) std::cout << "BTN_BACK" << std::endl;
#endif
                            buttons[BTN_BACK_POS] = val;
                            break;
                       // support Wacom INTOUS S
                        case BTN_0:
#ifdef DEBUG_BUTTON_READ
                            if(val) std::cout << "BTN_0" << std::endl;
#endif
                            buttons[BTN_LEFT_POS] = val;
                            break;
                        case BTN_1:
#ifdef DEBUG_BUTTON_READ
                            if(val) std::cout << "BTN_1" << std::endl;
#endif
                            buttons[BTN_RIGHT_POS] = val;
                            break;
                        case BTN_2:
#ifdef DEBUG_BUTTON_READ
                            if(val) std::cout << "BTN_2" << std::endl;
#endif
                            buttons[BTN_FORWARD_POS] = val;
                            break;
                        case BTN_3:
#ifdef DEBUG_BUTTON_READ
                            if(val) std::cout << "BTN_3" << std::endl;
#endif
                            buttons[BTN_BACK_POS] = val;
                            break;
                        }
                    }
                }
            }
        }

        pthread_mutex_unlock(&dataMutex);
        nanosleep(&tim,0);
        pthread_mutex_lock(&threadMutex);
    }
    //stop signal received, giving ack and exiting
    threadRunning = true;
    pthread_mutex_unlock(&threadMutex);
}

void* BambooHandler::staticReadThread(void *h) {
    ((BambooHandler*)h)->readThread();
    return 0;
}

int BambooHandler::eventFilter(const dirent *dir) {
    return strncmp(EVENT_DEV_NAME, dir->d_name, 5) == 0;
}

int BambooHandler::extractNumberFromEvent(char *name) {
    return atoi(name + 5);
}
