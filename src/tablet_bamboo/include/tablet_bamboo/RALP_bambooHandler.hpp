#ifndef RALP_BAMBOOHANDLER_HPP
#define RALP_BAMBOOHANDLER_HPP

#include "tablet_bamboo/bambooHandler.hpp"

class RALP_bambooHandler : public BambooHandler {
public:
    /** Enumerator type for rotation */
    enum Rotation{
        /** Device in the right direction */
        STRAIGHT,
        /** Device rotated clockwise by 90 degrees */
        ROT90DEG,
        /** Device rotated by 180 degrees */
        ROT180DEG,
        /** Device rotated counterclockwise by 90 degrees */
        ROT270DEG
    };

    /**
     * Change the rotation of the device
     * @param [in] rotation     Rotation to be set
     */
    void setRotate(Rotation rotation);

    /**
     * Scans all /dev/input/event*, and create an instance with the first Wacom Bamboo found
     *
     * @return An instance of the tablet or 0 if no instance could be created
     */
    static RALP_bambooHandler* firstDevice();

    /**
     * Get last normalized position. All output are optional, put 0 if not interesting.
     * Also, all values are scaled from 0.0 to 1.0 and rotated.
     *
     * @param [out] x  x position of the pen
     * @param [out] y  y position of the pen
     * @param [out] d  last distance of the pen if not touching, 0 else
     * @param [out] p  pressure of the pen if touching, 0 otherwise
     *
     * @return True if the pen is touching, false else
     */
    bool getNormPos(double* x = 0, double* y = 0, double* d = 0, double* p = 0);

protected:
    Rotation rotation;

    RALP_bambooHandler();
    virtual ~RALP_bambooHandler();

private:
    /** Copy constructor; not to be implemented or used */
    RALP_bambooHandler(const RALP_bambooHandler& other);
    /** Assignment operator; not to be implemented or used */
    RALP_bambooHandler& operator= (RALP_bambooHandler& other);
};

#endif // RALP_BAMBOOHANDLER_HPP
