Title      : ralp_msgs README file
Author     : Jesus Ortiz (jesus.ortiz@iit.it)
Description: This file contains detailed information about the contents of the
             ralp_msgs ROS package
             Please keep it updated if you modify any action, service or message

Actions
-------

  [endoscope_control.action]

This actionlib service receives the endoscope position in two angles (theta,
phi) and the velocity (velocity), with one single value for both axis. The
result of the action is given only in angular position (theta, phi) as well as
the feedback.

  * Goal

    float32 theta
    float32 phi
    float32 velocity

  * Result

    float32 theta
    float32 phi

  * Feedback

    float32 theta
    float32 phi


  [laser_control.action]

This actionlib service receives the micro robot position in two angles (alpha,
beta). The result of the action is given in angular position (alpha, beta) as
well as the feedback.

  * Goal

    float32 alpha
    float32 beta

  * Result

    float32 alpha
    float32 beta

  * Feedback

    float32 alpha
    float32 beta


  [laser_activation.action]

This actionlib service receives the laser activation (L). The result and
feedback is the same variable (L).

  * Goal

    bool L

  * Result

    bool L

  * Feedback

    bool L


  [laser_transform.action]

This actionlib service transforms from the input command from the input device
to the micro robot coordinates. To do that it uses the stored calibration data.
This calibration data depends on the camera and on the microrobot. The
calibration procedure it's done by the "Main UI".

  * Goal

    float32 x
    float32 y

  * Result

    float32 x
    float32 y

  * Feedback

    float32 x
    float32 y


Services
--------

  [camAttr.srv]

This service is used to modify the camera attributes. The request includes an
operation code (op) and attribute id (id) and a list of possible values
(value_bool, value_uint, value_int, value_float, value_string). The answer
includes an error flag (err) and the same list of possible values.

  * Request

    uint8   op
    uint16  id
    bool    value_bool
    uint64  value_uint
    int64   value_int
    float32 value_float
    string  value_string

  * Answer

    bool    err
    bool    value_bool
    uint64  value_uint
    int64   value_int
    float32 value_float
    string  value_string

The possible operations (op) are:

  * GET_NUMBER   Get the number of attributes (returned in "value_uint")
  * GET_NAME     Get the name of an attribute (returned in "value_string")
  * GET_CATEGORY Get a string with the category classification. Each level in
                 the category is separated by a "/" (returned in "value_string")
  * GET_TYPE     Get the type of an attribute (returned in "value_uint")
  * GET_FLAGS    Get the flags of an attribute . The flags are: READ, WRITE,
                 VOLATILE and CONST (returned in "value_uint")
  * GET_MIN      Get minimum value of an attribute. Valid only for "value_uint"
                 "value_int" and "value_float".
  * GET_MAX      Get maximum value of an attribute. Valid only for "value_uint"
                 "value_int" and "value_float". The special case of an
                 enumeration attribute, returns the number of possible values in
                 "value-uint".
  * GET_ENUM     Get an enumeration value corresponding with the "value_uint"
                 slot within the possible values (returned in "value_string")
  * GET          Get the value of an attribute
  * SET          Set the value on an attribute


  [laserAttr.srv]

This service is the same as the camera attributes one.


  [endoscope_motors.srv]

This service enables/disables the motors of the endoscope with a single boolean
value.

  * Request

    bool motors

  * Answer

    bool motors


  [laser_calibration.srv]

This service is use for the micro robot calibration.

  * Request

    uint8 op
	string calName
	int8 calPointId

  * Answer

    bool err
	int8 calPointId
	
The possible operations (op) are:

  * OP_LOAD_CALIB		Load a calibration file.
  * OP_SAVE_CALIB		Save the current calibration on a file.
  * OP_START_CALIB		Start calibration procedure.
  * OP_ABORT_CALIB		Abort calibration procedure.
  * OP_NEXT_POINT		Next calibration point. Ask the server to move the laser
                        to the corresponding calibration point.
  * OP_SET_POINT		Set calibration point. The server adds a new calibration
                        point and moves to the next calibration point.
  * OP_RUN_TRANSFORM	Run transformation. The calibration is ready and we can
                        start sending commands to the laser.
                        
If the calibration is saved in a file:

   1 Client asks to load a calibration file:
       op = OP_LOAD_CALIB
       calName = <name of the calibration file to load>
   2 Server answers ok:
       err = false
   3 Client asks to run transformation:
       op = OP_RUN_TRANSFORM
   4 Server answers ok:
       err = false

Calibration procedure:
   
   1 Client starts calibration mode:
       op = OP_START_CALIB
   2 Server answers ok:
       err = false
   3 Client ask for first calibration point:
       op = OP_NEXT_POINT
       calPointId = 0
   4 Server moves the laser to the first position, and sends ok:
      err = false
      calPointId = 0
   5 Client points to the coordinate (the input device is publishing the
     position):
       op = OP_SET_POINT
       calPointId = 0
   6 Server stores the new coordinates, the one from the input device, and the
     feedback of the laser. Then answers ok and moves to next point:
       err = false
       calPointId = 1
  7a If the calibration is not finished, goto point 3
  7b If the calibration is finished, the server sends:
       err = false
       calPointId = 0
   
Notes:

  * The calibration patterns might be passed to the laser transormation with
    the node parameters.


Messages
--------

The images are always send using "image_transport" messages.


  [cog_info.msg]

This message is periodically published by the cognitive supervisor.
It contains an estimation of the depth of ablation achieved during
a laser cut.

  * Message

    Header header
    float32 depth


  [alert.msg]

The cognitive supervisor publishes alerts that are read and display by the main
UI. The alert has a level of warning, the code and the description.

  * Message

    Header header
    uint16 level
    uint16 code
    string description


  [input_device.msg]

The input device cast the coordinates (x, y), the buttons (maximum 8 buttons
stored in a byte) and pressure.

  * Message

    Header header
    float32 x
    float32 y
    uint8   buttons
    float32 pressure

