#include "tablet_bamboo/RALP_bambooHandler.hpp"

void RALP_bambooHandler::setRotate(Rotation rotation) {
    this->rotation = rotation;
}

RALP_bambooHandler* RALP_bambooHandler::firstDevice(){
    static RALP_bambooHandler bamboo;
    if(bamboo.isInitialized() || bamboo.init())
        return &bamboo;
    else
        return 0;
}

bool RALP_bambooHandler::getNormPos(double* x, double* y, double* d, double* p) {
    if(!isInitialized())
        return false;

    if((x!=0)||(y!=0)){
        int intX, intY, xMax, yMax;

        if(!getPosition(&intX, &intY, &xMax, &yMax))
            return false;

        if(x!=0)
            switch(rotation) {
            case STRAIGHT:
                *x = ((double)intX) / xMax;
                break;
            case ROT90DEG:
                *x = 1.0 - ((double)intY) / yMax;
                break;
            case ROT180DEG:
                *x = 1.0 - ((double)intX) / xMax;
                break;
            case ROT270DEG:
                *x = ((double)intY) / yMax;
                break;
            }
        if(y!=0)
            switch(rotation) {
            case STRAIGHT:
                *y = ((double)intY) / yMax;
                break;
            case ROT90DEG:
                *y = ((double)intX) / xMax;
                break;
            case ROT180DEG:
                *y = 1.0 - ((double)intY) / yMax;
                break;
            case ROT270DEG:
                *y = 1.0 -((double)intX) / xMax;
                break;
            }
    }

    if(d!=0) {
        int intD, intDMax;

        if(!getDistance(&intD, &intDMax))
            return false;

        *d = ((double)intD) / intDMax;
    }

    if(p!=0) {
        int intP, intPMax;

        if(!getPressure(&intP, &intPMax))
            return false;

        *p = ((double)intP) / intPMax;
    }

    return true;
}

RALP_bambooHandler::RALP_bambooHandler() :
    BambooHandler(true, true) {
    rotation = STRAIGHT;
}

RALP_bambooHandler::~RALP_bambooHandler() {
}
