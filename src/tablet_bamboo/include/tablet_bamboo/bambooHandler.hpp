#ifndef BAMBOOHANDLER_HPP
#define BAMBOOHANDLER_HPP

/*
 * To use programs derived on this class without being root, access to the device
 * must be granted from udev. The easiest way to accomplish this is to get the
 * device idVendor and idProduct by connecting it and giving immediately a dmesg
 * command in the shell. Then create a new rule, for example
 * /etc/udev/rules.d/10-bamboo.rules
 * and use it to define access mode, for example
 * ATTRS{idProduct}=="00de",ATTRS{idVendor}=="056a",MODE:="0666"
 * where 056a (Wacom Co.,Ltd) and 00de (CTH-470) are the code shown in dmesg
 * and 666 stands for read-write for user root, group root, and other users.
 * Note that := after MODE is mandatory after Ubuntu 14.04 and that you will
 * need to reboot to let changes take effect.
 */

/*
 * Note that for new devices like the Intuos S you might have to update the wacom drivers. Therefore,
 * follow the instructions in the github repository of the input-wacom project:
 * https://github.com/linuxwacom/input-wacom/wiki/Installing-input-wacom-from-source
 * For connection via bluetooth, the udev rule might not work and you have to use a hacky rule
 * like the following:
 * DRIVERS=="wacom",MODE="0666",GROUP="plugdev"
 * which sets the mode of every wacom device to 666.
 * If this is fixed in future, a rule like the following should work:
 * ATTRS{idProduct}=="0376",ATTRS{idVendor}=="056a",MODE:="0666",GROUP="plugdev"
 * ATTRS{idProduct}=="0377",ATTRS{idVendor}=="056a",MODE:="0666",GROUP="plugdev"
 * with id 0376 for usb and id 0377 for bluetooth connection.
 * @ Jan Krieglstein, 06.02.2019
 * */

#include <vector>
#include <pthread.h>

struct dirent;
struct input_absinfo;

/** This is the basic class to handle bamboo tablets */
class BambooHandler {
public:
    /**
     * Constructor
     *
     * @param[in] pen      If true pen device is used for positioning and click, else finger
     * @param[in] disable  If true the tablet will not be usable in the operative system
     *                     after init and before deInit (or evenctually destruction)
     */
    BambooHandler(bool pen = true, bool disable = true);

    /** Destructor */
    virtual ~BambooHandler();

    /**
     * Try to initialize searching among devices
     *
     * @return True on success, false on failure
     */
    bool init();

    /**
     * Try to initialize with given devices in /dev/input/event*.
     * The use of this method is discouraged, in favour of init() without parameters.
     *
     * @param events  Array with events in /dev/input; for each N in this array
     *                /dev/input/eventN will be opened. Element at 0 is the device
     *                for positioning and click.
     *
     * @return True on success, false on failure
     */
    bool init(std::vector<int> events);

    /** Free resources. This is automatically called by destructor. */
    void deInit();

    /**
     * Get last position of the device (pen or touch) with respect to upper-left
     * boundary; all output are optional, put 0 if not needed
     *
     * @param[out] xVal  X position
     * @param[out] yVal  Y position
     * @param[out] xMax  X upper bound
     * @param[out] yMax  Y upper bound
     * @param[out] xMin  X lower bound
     * @param[out] yMin  Y lower bound
     * @param[out] xRes  X resolution (points/mm)
     * @param[out] yRes  Y resolution (points/mm)
     * @return True if device was initialized and values are output, false otherwise
     */
    bool getPosition(int* xVal = 0, int* yVal = 0,
                     int* xMax = 0, int* yMax = 0, int* xMin = 0, int* yMin = 0,
                     int* xRes = 0, int* yRes = 0);

    /**
     * Get last distance; all output are optional, put 0 if not needed
     *
     * @param[out] distVal  Value
     * @param[out] distMax  Upper bound
     * @param[out] distMin  Lower bound
     * @param[out] distRes  Resolution (points/mm) (may be not available and output 0)
     * @return True if device was initialized and values are output, false otherwise
     */
    bool getDistance(int* distVal = 0, int* distMax = 0, int* distMin = 0, int* distRes = 0);

    /**
     * Get last pressure; all output are optional, put 0 if not needed
     *
     * @param[out] pressVal  Value
     * @param[out] pressMax  Upper bound
     * @param[out] pressMin  Lower bound
     * @param[out] pressRes  Resolution (may be not available and output 0)
     * @return True if device was initialized and values are output, false otherwise
     */
    bool getPressure(int* pressVal = 0, int* pressMax = 0, int* pressMin = 0, int* pressRes = 0);

    /**
     * Tell if device is touching (device need to be initialized)
     *
     * @return True if initialized and it's touching, false if it's not touching or it's
     * not initialized
     */
    bool isTouching();

    /**
     * Get the status of all the buttons. All output are optional, pass 0 if not needed
     *
     * @param[out] left        Left button
     * @param[out] right       Right button
     * @param[out] forward     Forward button
     * @param[out] backward    Backward button
     * @param[out] stylus1     Stylus button 1
     * @param[out] stylus2     Stylus button 2
     *
     * @return True if device was initialized and values are output, false otherwise
     */
    bool getButtonState(bool* left = 0, bool* right = 0, bool* forward = 0,
                        bool* backward = 0, bool* stylus1 = 0, bool* stylus2 = 0);

    /**
     * Tell if init process has been accomplished.
     *
     * @return True if class has been initialized, false otherwise
     */
    bool isInitialized();

protected:
    std::vector<int> devices; /**< After initialization, list of opened devices. */
    const bool disable; /**< If true the tablet will not be usable in the operative system after init and before deInit */
    const bool pen; /**< If true pen device is used for positioning and click, else finger */
    bool initialized; /**< True if class has been initialized, false otherwise */
    bool threadRunning; /**< Tell if the thread is running */

    input_absinfo* data[4]; /**< x, y, distance (if any) and pressure (if any), look at defines at cpp file */
    bool buttons[7]; /**< Pen/finger pressed, stylus 1, stylus 2, left, right, forward, backward, look at defines at cpp file */

    pthread_mutex_t dataMutex; /**< Mutex to access read data */
    pthread_mutex_t threadMutex; /**< Mutex to stop synchronously the thread */

    pthread_t readThreadH; /**< Handler for readThread */

    /** Thread running and reading inputs, non-static version */
    void readThread();

    /**
     * Thread running and reading inputs, static version
     * @param [in] h  Pointer to the class which is calling it
     *
     * @return Unused
     */
    static void* staticReadThread(void* h);

private:
    void* dpy; /**< X11 Display */
    void* deviceList; /**< X11 device list */
    int devNum; /**< Number of element in device list */
    unsigned long devEn; /**< "Device Enabled" property */

    /**
     * Used to filter file containing the expression "event" as first 5 letters
     *
     * @param[in] dir  Descriptor of the file
     *
     * @return A value != 0 if correct, 0 otherwise
     */
    inline static int eventFilter(const struct dirent* dir);

    /**
     * Extract the number from the event name
     *
     * @param[in]  name  Name of the file
     *
     * @return Output value
     */
    inline static int extractNumberFromEvent(char* name);

    /**
     * Copy constructor; not to be implemented or used
     * @param[in] other unused
     */
    BambooHandler(const BambooHandler& other);

    /**
     * Assignment operator; not to be implemented or used
     * @param[in] other unused
     * @return unused
     */
    BambooHandler& operator= (BambooHandler& other);
};

#endif // BAMBOOHANDLER_HPP
