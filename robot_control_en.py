#! /usr/bin/env python
# coding=utf-8
import time
import libpyauboi5
import logging
from logging.handlers import RotatingFileHandler
from multiprocessing import Process, Queue
import os
from math import pi

# Create a logger
#logger = logging.getLogger()

logger = logging.getLogger('main.robotcontrol')


def logger_init():
    # Log level master switch
    logger.setLevel(logging.INFO)

    # Create log directory
    if not os.path.exists('./logfiles'):
        os.mkdir('./logfiles')

    # Create a handler to write to the log file
    logfile ='./logfiles/robot-ctl-python.log'

    # Open the log file in append mode
    # fh = logging.FileHandler(logfile, mode='a')
    fh = RotatingFileHandler(logfile, mode='a', maxBytes=1024*1024*50, backupCount=30)

    # Output to file log level switch
    fh.setLevel(logging.INFO)

    # Create another handler for output to the console
    ch = logging.StreamHandler()

    # Output to the log level switch of the console
    ch.setLevel(logging.INFO)

    # Define the output format of the handler
    # formatter = logging.Formatter("%(asctime)s-%(filename)s[line:%(lineno)d]-%(levelname)s: %(message)s")
    formatter = logging.Formatter("%(asctime)s [%(thread)u] %(levelname)s: %(message)s")

    # Set the format for file output
    fh.setFormatter(formatter)

    # Console output setting format
    ch.setFormatter(formatter)

    # Set file output to logger
    logger.addHandler(fh)

    # Set console output to logger
    logger.addHandler(ch)


class RobotEventType:
    RobotEvent_armCanbusError = 0 # Robotic arm CAN bus error
    RobotEvent_remoteHalt = 1 # Robot stop
    RobotEvent_remoteEmergencyStop = 2 # Robot remote emergency stop
    RobotEvent_jointError = 3 # Joint error
    RobotEvent_forceControl = 4 # Force control
    RobotEvent_exitForceControl = 5 # Exit force control
    RobotEvent_softEmergency = 6 # Soft emergency stop
    RobotEvent_exitSoftEmergency = 7 # Exit soft emergency stop
    RobotEvent_collision = 8 # collision
    RobotEvent_collisionStatusChanged = 9 # Collision status changed
    RobotEvent_tcpParametersSucc = 10 # The tool dynamics parameters are set successfully
    RobotEvent_powerChanged = 11 # The state of the power switch of the robot arm changes
    RobotEvent_ArmPowerOff = 12 # Robot power is off
    RobotEvent_mountingPoseChanged = 13 # The installation location has changed
    RobotEvent_encoderError = 14 # Encoder error
    RobotEvent_encoderLinesError = 15 # The number of encoder lines is inconsistent
    RobotEvent_singularityOverspeed = 16 # Singularity Overspeed
    RobotEvent_currentAlarm = 17 # Robotic arm current is abnormal
    RobotEvent_toolioError = 18 # Robotic arm tool end error
    RobotEvent_robotStartupPhase = 19 # Robot start phase
    RobotEvent_robotStartupDoneResult = 20 # Robot start-up completion result
    RobotEvent_robotShutdownDone = 21 # Robot shutdown result
    RobotEvent_atTrackTargetPos = 22 # Signal notification of robot arm track movement in place
    RobotEvent_SetPowerOnDone = 23 # Set power status completed
    RobotEvent_ReleaseBrakeDone = 24 # Robotic arm brake release completed
    RobotEvent_robotControllerStateChaned = 25 # Robotic arm control state changes
    RobotEvent_robotControllerError = 26 # Robotic arm control error-usually returned when there is a problem with the algorithm planning
    RobotEvent_socketDisconnected = 27 # socket disconnected
    RobotEvent_overSpeed = 28  # overspeed
    RobotEvent_algorithmException = 29 # Robotic arm algorithm is abnormal
    RobotEvent_boardIoPoweron = 30 # External power-on signal
    RobotEvent_boardIoRunmode = 31 # linkage/manual
    RobotEvent_boardIoPause = 32 # External pause signal
    RobotEvent_boardIoStop = 33 # External stop signal
    RobotEvent_boardIoHalt = 34 # External shutdown signal
    RobotEvent_boardIoEmergency = 35 # External emergency stop signal
    RobotEvent_boardIoRelease_alarm = 36 # External alarm release signal
    RobotEvent_boardIoOrigin_pose = 37 # External return to origin signal
    RobotEvent_boardIoAutorun = 38 # External automatic run signal
    RobotEvent_safetyIoExternalEmergencyStope = 39 # External emergency stop input 01
    RobotEvent_safetyIoExternalSafeguardStope = 40 # External protection stop input 02
    RobotEvent_safetyIoReduced_mode = 41 # Reduced mode input
    RobotEvent_safetyIoSafeguard_reset = 42 # Protection reset
    RobotEvent_safetyIo3PositionSwitch = 43 # Three-state switch 1
    RobotEvent_safetyIoOperationalMode = 44 # Operation mode
    RobotEvent_safetyIoManualEmergencyStop = 45 # Teach pendant emergency stop 01
    RobotEvent_safetyIoSystemStop = 46 # System stop input
    RobotEvent_alreadySuspended = 47 # The robotic arm is suspended
    RobotEvent_alreadyStopped = 48 # Robot stop
    RobotEvent_alreadyRunning = 49 # Robotic arm running
    RobotEvent_MoveEnterStopState = 1300 #Movement enters the stop phase
    RobotEvent_None = 999999

    # Non-error events
    NoError = (RobotEvent_forceControl,
               RobotEvent_exitForceControl,
               RobotEvent_tcpParametersSucc,
               RobotEvent_powerChanged,
               RobotEvent_mountingPoseChanged,
               RobotEvent_robotStartupPhase,
               RobotEvent_robotStartupDoneResult,
               RobotEvent_robotShutdownDone,
               RobotEvent_SetPowerOnDone,
               RobotEvent_ReleaseBrakeDone,
               RobotEvent_atTrackTargetPos,
               RobotEvent_robotControllerStateChaned,
               RobotEvent_robotControllerError,
               RobotEvent_algorithmException,
               RobotEvent_alreadyStopped,
               RobotEvent_alreadyRunning,
               RobotEvent_boardIoPoweron,
               RobotEvent_boardIoRunmode,
               RobotEvent_boardIoPause,
               RobotEvent_boardIoStop,
               RobotEvent_boardIoHalt,
               RobotEvent_boardIoRelease_alarm,
               RobotEvent_boardIoOrigin_pose,
               RobotEvent_boardIoAutorun,
               RobotEvent_safetyIoExternalEmergencyStope,
               RobotEvent_safetyIoExternalSafeguardStope,
               RobotEvent_safetyIoReduced_mode,
               RobotEvent_safetyIoSafeguard_reset,
               RobotEvent_safetyIo3PositionSwitch,
               RobotEvent_safetyIoOperationalMode,
               RobotEvent_safetyIoManualEmergencyStop,
               RobotEvent_safetyIoSystemStop,
               RobotEvent_alreadySuspended,
               RobotEvent_alreadyStopped,
               RobotEvent_alreadyRunning,
               RobotEvent_MoveEnterStopState
               )
               
    UserPostEvent = (RobotEvent_robotControllerError,
                     RobotEvent_safetyIoExternalSafeguardStope,
                     RobotEvent_safetyIoSystemStop
                     )
    ClearErrorEvent = (RobotEvent_armCanbusError,
                       RobotEvent_remoteEmergencyStop,
                       RobotEvent_jointError,
                       RobotEvent_collision,
                       RobotEvent_collisionStatusChanged,
                       RobotEvent_encoderError,
                       RobotEvent_encoderLinesError,
                       RobotEvent_currentAlarm,
                       RobotEvent_softEmergency,
                       RobotEvent_exitSoftEmergency
                       )

    def __init__(self):
        pass


class RobotErrorType:
    RobotError_SUCC = 0 # No error
    RobotError_Base = 2000
    RobotError_RSHD_INIT_FAILED = RobotError_Base + 1 # Library initialization failed
    RobotError_RSHD_UNINIT = RobotError_Base + 2 # The library is not initialized
    RobotError_NoLink = RobotError_Base + 3 # No link
    RobotError_Move = RobotError_Base + 4 # Robotic arm movement error
    RobotError_ControlError = RobotError_Base + RobotEventType.RobotEvent_robotControllerError
    RobotError_LOGIN_FAILED = RobotError_Base + 5 # Robotic arm login failed
    RobotError_NotLogin = RobotError_Base + 6 # Robot is not logged in
    RobotError_ERROR_ARGS = RobotError_Base + 7 # Parameter error

    def __init__(self):
        pass


class RobotEvent:
    def __init__(self, event_type=RobotEventType.RobotEvent_None, event_code=0, event_msg=''):
        self.event_type = event_type
        self.event_code = event_code
        self.event_msg = event_msg


# noinspection SpellCheckingInspection
class RobotError(Exception):
    def __init__(self, error_type=RobotErrorType.RobotError_SUCC, error_code=0, error_msg=''):
        self.error_type = error_type
        self.error_cdoe = error_code
        self.error_msg = error_msg

    def __str__(self):
        return "RobotError type{0} code={1} msg={2}".format(self.error_type, self.error_cdoe, self.error_msg)


class RobotDefaultParameters:
    # Default kinetic parameters
    tool_dynamics = {"position": (0.0, 0.0, 0.0), "payload": 1.0, "inertia": (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)}

    # Default collision level
    collision_grade = 6

    def __init__(self):
        pass

    def __str__(self):
        return "Robot Default parameters, tool_dynamics:{0}, collision_grade:{1}".format(self.tool_dynamics,
                                                                                         self.collision_grade)


class RobotMoveTrackType:
    # Arc
    ARC_CIR = 2
    # Trajectory
    CARTESIAN_MOVEP = 3
    # The following four third-order spline interpolation curves have discontinuous accelerations at the start and end points, which are not suitable for the new joint drive version
    # Cubic spline interpolation (passing control points), automatically optimize the trajectory running time, currently does not support attitude changes
    CARTESIAN_CUBICSPLINE = 4
    # Need to set the time interval of three uniform B-spline interpolation (passing the control point), currently does not support attitude changes
    CARTESIAN_UBSPLINEINTP = 5
    # Third-order spline interpolation curve
    JIONT_CUBICSPLINE = 6
    # Can be used for track playback
    JOINT_UBSPLINEINTP = 7

    def __init__(self):
        pass


class RobotIOType:
    # Control cabinet IO
    ControlBox_DI = 0
    ControlBox_DO = 1
    ControlBox_AI = 2
    ControlBox_AO = 3
    # User IO
    User_DI = 4
    User_DO = 5
    User_AI = 6
    User_AO = 7

    def __init__(self):
        pass


class RobotToolIoName:
    tool_io_0 = "T_DI/O_00"
    tool_io_1 = "T_DI/O_01"
    tool_io_2 = "T_DI/O_02"
    tool_io_3 = "T_DI/O_03"

    tool_ai_0 = "T_AI_00"
    tool_ai_1 = "T_AI_01"

    def __init__(self):
        pass


class RobotUserIoName:
    # Control cabinet user DI
    user_di_00 = "U_DI_00"
    user_di_01 = "U_DI_01"
    user_di_02 = "U_DI_02"
    user_di_03 = "U_DI_03"
    user_di_04 = "U_DI_04"
    user_di_05 = "U_DI_05"
    user_di_06 = "U_DI_06"
    user_di_07 = "U_DI_07"
    user_di_10 = "U_DI_10"
    user_di_11 = "U_DI_11"
    user_di_12 = "U_DI_12"
    user_di_13 = "U_DI_13"
    user_di_14 = "U_DI_14"
    user_di_15 = "U_DI_15"
    user_di_16 = "U_DI_16"
    user_di_17 = "U_DI_17"

    # Control cabinet user Ｏ
    user_do_00 = "U_DO_00"
    user_do_01 = "U_DO_01"
    user_do_02 = "U_DO_02"
    user_do_03 = "U_DO_03"
    user_do_04 = "U_DO_04"
    user_do_05 = "U_DO_05"
    user_do_06 = "U_DO_06"
    user_do_07 = "U_DO_07"
    user_do_10 = "U_DO_10"
    user_do_11 = "U_DO_11"
    user_do_12 = "U_DO_12"
    user_do_13 = "U_DO_13"
    user_do_14 = "U_DO_14"
    user_do_15 = "U_DO_15"
    user_do_16 = "U_DO_16"
    user_do_17 = "U_DO_17"

    # Control cabinet analog IO
    user_ai_00 = "VI0"
    user_ai_01 = "VI1"
    user_ai_02 = "VI2"
    user_ai_03 = "VI3"

    user_ao_00 = "VO0"
    user_ao_01 = "VO1"
    user_ao_02 = "VO2"
    user_ao_03 = "VO3"

    def __init__(self):
        pass


class RobotStatus:
    # Robot arm is currently stopped
    Stopped = 0
    # The robotic arm is currently running
    Running = 1
    # The robotic arm is currently suspended
    Paused = 2
    # The robotic arm is currently restored
    Resumed = 3

    def __init__(self):
        pass


class RobotRunningMode:
    # Robotic arm simulation mode
    RobotModeSimulator = 0
    # Robotic arm real mode
    RobotModeReal = 1

    def __init__(self):
        pass


class RobotToolPowerType:
    OUT_0V = 0
    OUT_12V = 1
    OUT_24V = 2

    def __init__(self):
        pass


class RobotToolIoAddr:
    TOOL_DIGITAL_IO_0 = 0
    TOOL_DIGITAL_IO_1 = 1
    TOOL_DIGITAL_IO_2 = 2
    TOOL_DIGITAL_IO_3 = 3

    def __init__(self):
        pass


class RobotCoordType:
    # Base coordinate system
    Robot_Base_Coordinate = 0
    # End coordinate system
    Robot_End_Coordinate = 1
    # User coordinate system
    Robot_World_Coordinate = 2

    def __init__(self):
        pass


class RobotCoordCalMethod:
    CoordCalMethod_xOy = 0
    CoordCalMethod_yOz = 1
    CoordCalMethod_zOx = 2
    CoordCalMethod_xOxy = 3
    CoordCalMethod_xOxz = 4
    CoordCalMethod_yOyx = 5
    CoordCalMethod_yOyz = 6
    CoordCalMethod_zOzx = 7
    CoordCalMethod_zOzy = 8

    def __init__(self):
        pass


class RobotToolDigitalIoDir:
    # Enter
    IO_IN = 0
    # Output
    IO_OUT = 1

    def __init__(self):
        pass


class Auboi5Robot:
    # Number of clients
    __client_count = 0

    def __init__(self):
        self.rshd = -1
        self.connected = False
        self.last_error = RobotError()
        self.last_event = RobotEvent()
        self.atTrackTargetPos = False
        Auboi5Robot.__client_count += 1

    def __del__(self):
        Auboi5Robot.__client_count -= 1
        self.uninitialize()
        logger.info("client_count={0}".format(Auboi5Robot.__client_count))

    def __str__(self):
        return "RSHD={0}, connected={1}".format(self.rshd, self.connected)

    @staticmethod
    def get_local_time():
        """"
        * FUNCTION: get_local_time
        * DESCRIPTION: Get the current time of the system
        * INPUTS: No input
        * OUTPUTS:
        * RETURNS: Output the current time string of the system
        * NOTES:
        """
        return time.strftime("%b %d %Y %H:%M:%S", time.localtime(time.time()))

    def robot_event_callback(self, event):
        """"
        * FUNCTION: robot_event_callback
        * DESCRIPTION: Robotic arm event
        * INPUTS: No input
        * OUTPUTS:
        * RETURNS: system event callback function
        * NOTES:
        """
        print("event={0}".format(event))
        if event['type'] not in RobotEventType.NoError:
            self.last_error = RobotError(event['type'], event['code'], event['content'])
        else:
            self.last_event = RobotEvent(event['type'], event['code'], event['content'])

    @staticmethod
    def raise_error(error_type, error_code, error_msg):
        """"
        * FUNCTION: raise_error
        * DESCRIPTION: Throw an exception event
        * INPUTS: No input
        * OUTPUTS:
        * RETURNS: None
        * NOTES:
        """
        raise RobotError(error_type, error_code, error_msg)

    def check_event(self):
        """"
        * FUNCTION: check_event
        * DESCRIPTION: Check whether an abnormal event has occurred in the robotic arm
        * INPUTS: input
        * OUTPUTS: output
        * RETURNS: void
        * NOTES: If an abnormal event is received, the function throws an abnormal event
        """
        if self.last_error.error_type != RobotErrorType.RobotError_SUCC:
            raise self.last_error
        if self.rshd == -1 or not self.connected:
            self.raise_error(RobotErrorType.RobotError_NoLink, 0, "no socket link")

    @staticmethod
    def initialize():
        """"
        * FUNCTION: initialize
        * DESCRIPTION: Initialize the robotic arm control library
        * INPUTS:
        * OUTPUTS:
        * RETURNS: Successful return: RobotError.RobotError_SUCC
        * Failure return: other
        * NOTES:
        """
        result = libpyauboi5.initialize()
        if result == RobotErrorType.RobotError_SUCC:
            return RobotErrorType.RobotError_SUCC
        else:
            return RobotErrorType.RobotError_RSHD_INIT_FAILED

    @staticmethod
    def uninitialize():
        """"
        * FUNCTION: uninitialize
        * DESCRIPTION: De-initialize the robotic arm control library
        * INPUTS: input
        * OUTPUTS: output
        * RETURNS: Successful return: RobotError.RobotError_SUCC
        * Failure return: other
        * NOTES:
        """
        return libpyauboi5.uninitialize()

    def create_context(self):
        """"
        * FUNCTION: create_context
        * DESCRIPTION: Create a robot arm control context handle
        * INPUTS:
        * OUTPUTS:
        * RETURNS: Successful return: RSHD
        * NOTES:
        """
        self.rshd = libpyauboi5.create_context()
        return self.rshd

    def get_context(self):
        """"
        * FUNCTION: get_context
        * DESCRIPTION: Get the current control context of the robotic arm
        * INPUTS:
        * OUTPUTS:
        * RETURNS: Context handle RSHD
        * NOTES:
        """
        return self.rshd

    def connect(self, ip='localhost', port=8899):
        """"
        * FUNCTION: connect
        * DESCRIPTION: Link to the robotic arm server
        * INPUTS: ip robotic arm server address
        * port port number
        * OUTPUTS:
        * RETURNS: Successful return: RobotError.RobotError_SUCC
        * Failure return: other
        *
        * NOTES:
        """
        logger.info("ip={0}, port={1}".format(ip, port))
        if self.rshd >= 0:
            if not self.connected:
                if libpyauboi5.login(self.rshd, ip, port) == 0:
                    self.connected = True
                    time.sleep(0.5)
                    return RobotErrorType.RobotError_SUCC
                else:
                    logger.error("login failed!")
                    return RobotErrorType.RobotError_LOGIN_FAILED
            else:
                logger.info("already connected.")
                return RobotErrorType.RobotError_SUCC
        else:
            logger.error("RSHD uninitialized!!!")
            return RobotErrorType.RobotError_RSHD_UNINIT

    def disconnect(self):
        """"
         * FUNCTION: disconnect
         * DESCRIPTION: Disconnect the robotic arm server link
         * INPUTS:
         * OUTPUTS:
         * RETURNS: Successful return: RobotError.RobotError_SUCC
         * Failure return: other
         * NOTES:
         """
        if self.rshd >= 0 and self.connected:
            libpyauboi5.logout(self.rshd)
            self.connected = False
            time.sleep(0.5)
            return RobotErrorType.RobotError_SUCC
        else:
            logger.warn("RSHD uninitialized or not login!!!")
            return RobotErrorType.RobotError_NotLogin

    def robot_startup(self, collision=RobotDefaultParameters.collision_grade,
                      tool_dynamics=RobotDefaultParameters.tool_dynamics):
        """
        * FUNCTION: robot_startup
        * DESCRIPTION: Start the robotic arm
        * INPUTS: collision: collision level range (0~10) default: 6
        * tool_dynamics: kinematics parameters
        * tool_dynamics = position, unit (m): {"position": (0.0, 0.0, 0.0),
        * Payload, unit (kg): "payload": 1.0,
        * Inertia: "inertia": (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)}
        *
        * OUTPUTS:
        * RETURNS: Successful return: RobotError.RobotError_SUCC
        * Failure return: other
        * NOTES:
        """
        self.check_event()
        if self.rshd >= 0 and self.connected:
            result = libpyauboi5.robot_startup(self.rshd, collision, tool_dynamics)
            time.sleep(0.5)
            return result
        else:
            logger.warn("RSHD uninitialized or not login!!!")
            return RobotErrorType.RobotError_NotLogin

    def robot_shutdown(self):
        """
        * FUNCTION: robot_shutdown
        * DESCRIPTION: Turn off the robotic arm
        * INPUTS:
        * OUTPUTS:
        * RETURNS: Successful return: RobotError.RobotError_SUCC
        * Failure return: other
        * NOTES:
        """
        if self.rshd >= 0 and self.connected:
            result = libpyauboi5.robot_shutdown(self.rshd)
            time.sleep(0.5)
            return result
        else:
            logger.warn("RSHD uninitialized or not login!!!")
            return RobotErrorType.RobotError_NotLogin

    def enable_robot_event(self):
        self.check_event()
        if self.rshd >= 0 and self.connected:
            self.set_robot_event_callback(self.robot_event_callback)
            return RobotErrorType.RobotError_SUCC
        else:
            logger.warn("RSHD uninitialized or not login!!!")
            return RobotErrorType.RobotError_NotLogin

    def init_profile(self):
        """"
        * FUNCTION: init_profile
        * DESCRIPTION: Initialize the global properties of the robotic arm control
        * INPUTS:
        * OUTPUTS:
        * RETURNS: Successful return: RobotError.RobotError_SUCC
        * Failure return: other
        *
        * NOTES: After the call is successful, the system will automatically clean up the previously set user coordinate system,
        * Speed, acceleration and other attributes
        """
        if self.rshd >= 0 and self.connected:
            return libpyauboi5.init_global_move_profile(self.rshd)
        else:
            logger.warn("RSHD uninitialized or not login!!!")
            return RobotErrorType.RobotError_NotLogin

    def set_joint_maxacc(self, joint_maxacc=(1.0, 1.0, 1.0, 1.0, 1.0, 1.0)):
        """
        * FUNCTION: set_joint_maxacc
        * DESCRIPTION: Set the maximum acceleration of six joints
        * INPUTS: joint_maxacc: the maximum acceleration of the six joints, unit (rad/s)
        * OUTPUTS:
        * RETURNS: Successful return: RobotError.RobotError_SUCC
        * Failure return: other
        * NOTES:
        """
        self.check_event()
        if self.rshd >= 0 and self.connected:
            return libpyauboi5.set_joint_maxacc(self.rshd, joint_maxacc)
        else:
            logger.warn("RSHD uninitialized or not login!!!")
            return RobotErrorType.RobotError_NotLogin

    def get_joint_maxacc(self):
        """U_DO_00
        * FUNCTION: get_joint_maxacc
        * DESCRIPTION: Get the maximum acceleration of six joints
        * INPUTS:
        * OUTPUTS:
        * RETURNS: Successful return: Maximum acceleration unit of six joints (rad/s^2)
        * Failure return: None
        * NOTES:
        """
        self.check_event()
        if self.rshd >= 0 and self.connected:
            return libpyauboi5.get_joint_maxacc(self.rshd)
        else:
            logger.warn("RSHD uninitialized or not login!!!")
            return None

    def set_joint_maxvelc(self, joint_maxvelc=(1.0, 1.0, 1.0, 1.0, 1.0, 1.0)):
        """
        * FUNCTION: set_joint_maxvelc
        * DESCRIPTION: Set the maximum speed of six joints
        * INPUTS: joint_maxvelc: maximum speed of six joints, unit (rad/s)
        * OUTPUTS:
        * RETURNS: Successful return: RobotError.RobotError_SUCC
        * Failure return: other
        * NOTES:
        """
        self.check_event()
        if self.rshd >= 0 and self.connected:
            return libpyauboi5.set_joint_maxvelc(self.rshd, joint_maxvelc)
        else:
            logger.warn("RSHD uninitialized or not login!!!")
            return RobotErrorType.RobotError_NotLogin

    def get_joint_maxvelc(self):
        """
        * FUNCTION: get_joint_maxvelc
        * DESCRIPTION: Get the maximum speed of six joints
        * INPUTS:
        * OUTPUTS:
        * RETURNS: Successful return: Maximum speed of six joints (rad/s)
        * Failure return: None
        * NOTES:
        """
        self.check_event()
        if self.rshd >= 0 and self.connected:
            return libpyauboi5.get_joint_maxvelc(self.rshd)
        else:
            logger.warn("RSHD uninitialized or not login!!!")
            return None

    def set_end_max_line_acc(self, end_maxacc=0.1):
        """
        * FUNCTION: set_end_max_line_acc
        * DESCRIPTION: Set the maximum linear acceleration at the end of the robotic arm
        * INPUTS: end_maxacc: the maximum acceleration speed at the end, unit (m/s^2)
        * OUTPUTS:
        * RETURNS: Successful return: RobotError.RobotError_SUCC
        * Failure return: other
        * NOTES:
        """
        self.check_event()
        if self.rshd >= 0 and self.connected:
            return libpyauboi5.set_end_max_line_acc(self.rshd, end_maxacc)
        else:
            logger.warn("RSHD uninitialized or not login!!!")
            return RobotErrorType.RobotError_NotLogin

    def get_end_max_line_acc(self):
        """
        * FUNCTION: get_end_max_line_acc
        * DESCRIPTION: Get the maximum linear acceleration at the end of the robotic arm
        * INPUTS:
        * OUTPUTS:
        * RETURNS: Successful return: the maximum acceleration at the end of the robot arm, unit (m/s^2)
        * Failure return: None
        * NOTES:
        """
        self.check_event()
        if self.rshd >= 0 and self.connected:
            return libpyauboi5.get_end_max_line_acc(self.rshd)
        else:
            logger.warn("RSHD uninitialized or not login!!!")
            return None

    def set_end_max_line_velc(self, end_maxvelc=0.1):
        """
        * FUNCTION: set_end_max_line_velc
        * DESCRIPTION: Set the maximum linear speed at the end of the robotic arm
        * INPUTS: end_maxacc: the maximum linear velocity at the end, unit (m/s)
        * OUTPUTS:
        * RETURNS: Successful return: RobotError.RobotError_SUCC
        * Failure return: other
        * NOTES:
        """
        self.check_event()
        if self.rshd >= 0 and self.connected:
            return libpyauboi5.set_end_max_line_velc(self.rshd, end_maxvelc)
        else:
            logger.warn("RSHD uninitialized or not login!!!")
            return RobotErrorType.RobotError_NotLogin

    def get_end_max_line_velc(self):
        """
        * FUNCTION: get_end_max_line_velc
        * DESCRIPTION: Get the maximum linear velocity at the end of the robotic arm
        * INPUTS:
        * OUTPUTS:
        * RETURNS: Successful return: Maximum speed at the end of the robotic arm, unit (m/s)
        * Failure return: None
        * NOTES:
        """
        self.check_event()
        if self.rshd >= 0 and self.connected:
            return libpyauboi5.get_end_max_line_velc(self.rshd)
        else:
            logger.warn("RSHD uninitialized or not login!!!")
            return None

    def set_end_max_angle_acc(self, end_maxacc=0.1):
        """
        * FUNCTION: set_end_max_angle_acc
        * DESCRIPTION: Set the maximum angular acceleration at the end of the robotic arm
        * INPUTS: end_maxacc: the maximum acceleration at the end, unit (rad/s^2)
        * OUTPUTS:
        * RETURNS: Successful return: RobotError.RobotError_SUCC
        * Failure return: other
        * NOTES:
        """
        self.check_event()
        if self.rshd >= 0 and self.connected:
            return libpyauboi5.set_end_max_angle_acc(self.rshd, end_maxacc)
        else:
            logger.warn("RSHD uninitialized or not login!!!")
            return RobotErrorType.RobotError_NotLogin

    def get_end_max_angle_acc(self):
        """
        * FUNCTION: get_end_max_angle_acc
        * DESCRIPTION: Get the maximum angular acceleration at the end of the robotic arm
        * INPUTS:
        * OUTPUTS:
        * RETURNS: Successful return: the maximum angular acceleration at the end of the robot arm, unit (m/s^2)
        * Failure return: None
        * NOTES:
        """
        self.check_event()
        if self.rshd >= 0 and self.connected:
            return libpyauboi5.get_end_max_angle_acc(self.rshd)
        else:
            logger.warn("RSHD uninitialized or not login!!!")
            return None

    def set_end_max_angle_velc(self, end_maxvelc=0.1):
        """
        * FUNCTION: set_end_max_angle_velc
        * DESCRIPTION: Set the maximum angular velocity at the end of the robotic arm
        * INPUTS: end_maxacc: the maximum speed at the end, unit (rad/s)
        * OUTPUTS:
        * RETURNS: Successful return: RobotError.RobotError_SUCC
        * Failure return: other
        * NOTES:
        """
        self.check_event()
        if self.rshd >= 0 and self.connected:
            return libpyauboi5.set_end_max_line_velc(self.rshd, end_maxvelc)
        else:
            logger.warn("RSHD uninitialized or not login!!!")
            return RobotErrorType.RobotError_NotLogin

    def get_end_max_angle_velc(self):
        """
        * FUNCTION: get_end_max_angle_velc
        * DESCRIPTION: Get the maximum angular velocity at the end of the robotic arm
        * INPUTS:
        * OUTPUTS:
        * RETURNS: Successful return: Maximum speed at the end of the robotic arm, unit (rad/s)
        * Failure return: None
        * NOTES:
        """
        self.check_event()
        if self.rshd >= 0 and self.connected:
            return libpyauboi5.get_end_max_line_velc(self.rshd)
        else:
            logger.warn("RSHD uninitialized or not login!!!")
            return None

    #zhar2_rem
    def move_to_target_in_cartesian(self, pos, rpy_xyz):
        """
        * FUNCTION: move_to_target_in_cartesian
        * DESCRIPTION: Given the Cartesian coordinate value and Euler angle, the manipulator axis moves to the target position and attitude
        * INPUTS: pos: position coordinates (x, y, z), unit (m)
        * rpy: Euler angle (rx, ry, rz), unit (degree)
        * OUTPUTS:
        * RETURNS: Successful return: RobotError.RobotError_SUCC
        * Failure return: other
        * NOTES:
        """
        self.check_event()
        if self.rshd >= 0 and self.connected:
            # Degrees -> radians
            rpy_xyz = [i / 180.0 * pi for i in rpy_xyz]
            # Euler angle to quaternion
            ori = libpyauboi5.rpy_to_quaternion(self.rshd, rpy_xyz)

            # Inverse calculation to get the joint angle
            joint_radian = libpyauboi5.get_current_waypoint(self.rshd)

            ik_result = libpyauboi5.inverse_kin(self.rshd, joint_radian['joint'], pos, ori)

            logging.info("ik_result====>{0}".format(ik_result))
            
            # Axis moves to the target position
            result = libpyauboi5.move_joint(self.rshd, ik_result["joint"])
            if result != RobotErrorType.RobotError_SUCC:
                self.raise_error(RobotErrorType.RobotError_Move, result, "move error")
            else:
                return RobotErrorType.RobotError_SUCC
        else:
            logger.warn("RSHD uninitialized or not login!!!")
            return RobotErrorType.RobotError_NotLogin

    def move_joint(self, joint_radian=(0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000), issync=True):
        """
        * FUNCTION: move_joint
        * DESCRIPTION: Robotic arm axis moves
        * INPUTS: joint_radian: joint angle of six joints, unit (rad)
        * OUTPUTS:
        * RETURNS: Successful return: RobotError.RobotError_SUCC
        * Failure return: other
        * NOTES:
        """
        self.check_event()
        if self.rshd >= 0 and self.connected:
            result = libpyauboi5.move_joint(self.rshd, joint_radian, issync)
            if result != RobotErrorType.RobotError_SUCC:
                self.raise_error(RobotErrorType.RobotError_Move, result, "move error")
            else:
                return RobotErrorType.RobotError_SUCC
        else:
            logger.warn("RSHD uninitialized or not login!!!")
            return RobotErrorType.RobotError_NotLogin

    # zhar2_rem
    def move_line(self, joint_radian=(0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000)):
        """
        * FUNCTION: move_line
        * DESCRIPTION: The robot arm maintains the current posture and moves in a straight line
        * INPUTS: joint_radian: joint angle of six joints, unit (rad)
        * OUTPUTS:
        * RETURNS: Successful return: RobotError.RobotError_SUCC
        * Failure return: other
        * NOTES:
        """
        self.check_event()
        if self.rshd >= 0 and self.connected:
            result = libpyauboi5.move_line(self.rshd, joint_radian)
            if result != RobotErrorType.RobotError_SUCC:
                self.raise_error(RobotErrorType.RobotError_Move, result, "move error")
            else:
                return RobotErrorType.RobotError_SUCC
        else:
            logger.warn("RSHD uninitialized or not login!!!")
            return RobotErrorType.RobotError_NotLogin

    def move_rotate(self, user_coord, rotate_axis, rotate_angle):
        """
        * FUNCTION: move_rotate
        * DESCRIPTION: Keep the current position and change the posture for rotation
        * INPUTS: user_coord: user coordinate system
        * user_coord = {'coord_type': 2,
        *'calibrate_method': 0,
        *'calibrate_points':
        * {"point1": (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        * "point2": (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        * "point3": (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)},
        *'tool_desc':
        * {"pos": (0.0, 0.0, 0.0),
        * "ori": (1.0, 0.0, 0.0, 0.0)}
        *}
        * rotate_axis: rotation axis (x,y,z) For example: (1,0,0) means to rotate along the Y axis
        * rotate_angle: rotation angle unit (rad)
        * OUTPUTS:
        * RETURNS: Successful return: RobotError.RobotError_SUCC
        * Failure return: other
        * NOTES:
        """
        self.check_event()
        if self.rshd >= 0 and self.connected:
            return libpyauboi5.move_rotate(self.rshd, user_coord, rotate_axis, rotate_angle)
        else:
            logger.warn("RSHD uninitialized or not login!!!")
            return RobotErrorType.RobotError_NotLogin

    def clear_offline_track(self):
        """
        * FUNCTION: clear_offline_track
        * DESCRIPTION: Clean up the non-online trajectory movement data on the server
        * INPUTS:
        * OUTPUTS:
        * RETURNS: Successful return: RobotError.RobotError_SUCC
        * Failure return: other
        * NOTES:
        """
        self.check_event()
        if self.rshd >= 0 and self.connected:
            return libpyauboi5.clear_offline_track(self.rshd)
        else:
            logger.warn("RSHD uninitialized or not login!!!")
            return RobotErrorType.RobotError_NotLogin

    def append_offline_track_waypoint(self, waypoints):
        """
        * FUNCTION: append_offline_track_waypoint
        * DESCRIPTION: Add non-online trajectory movement waypoints to the server
        * INPUTS: waypoints non-online trajectory movement waypoint originator (can contain less than 3000 waypoints), unit: radians
        * For example: ((0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        * (0.0, -0.000001, -0.000001, 0.000001, -0.000001, 0.0))
        * OUTPUTS:
        * RETURNS: Successful return: RobotError.RobotError_SUCC
        * Failure return: other
        * NOTES:
        """
        self.check_event()
        if self.rshd >= 0 and self.connected:
            return libpyauboi5.append_offline_track_waypoint(self.rshd, waypoints)
        else:
            logger.warn("RSHD uninitialized or not login!!!")
            return RobotErrorType.RobotError_NotLogin

    def append_offline_track_file(self, track_file):
        """
        * FUNCTION: append_offline_track_file
        * DESCRIPTION: Add a non-online track motion waypoint file to the server
        * INPUTS: The full path of the waypoint file, each line of the waypoint file contains the joint angles (radians) of six joints, separated by commas
        * OUTPUTS:
        * RETURNS: Successful return: RobotError.RobotError_SUCC
        * Failure return: other
        * NOTES:
        """
        self.check_event()
        if self.rshd >= 0 and self.connected:
            return libpyauboi5.append_offline_track_file(self.rshd, track_file)
        else:
            logger.warn("RSHD uninitialized or not login!!!")
            return RobotErrorType.RobotError_NotLogin

    def startup_offline_track(self):
        """
        * FUNCTION: startup_offline_track
        * DESCRIPTION: Notify the server to start non-online trajectory movement
        * INPUTS:
        * OUTPUTS:
        * RETURNS: Successful return: RobotError.RobotError_SUCC
        * Failure return: other
        * NOTES:
        """
        self.check_event()
        if self.rshd >= 0 and self.connected:
            return libpyauboi5.startup_offline_track(self.rshd)
        else:
            logger.warn("RSHD uninitialized or not login!!!")
            return RobotErrorType.RobotError_NotLogin

    def stop_offline_track(self):
        """
        * FUNCTION: stop_offline_track
        * DESCRIPTION: Notify the server to stop non-online track movement
        * INPUTS:
        * OUTPUTS:
        * RETURNS: Successful return: RobotError.RobotError_SUCC
        * Failure return: other
        * NOTES:
        """
        self.check_event()
        if self.rshd >= 0 and self.connected:
            return libpyauboi5.stop_offline_track(self.rshd)
        else:
            logger.warn("RSHD uninitialized or not login!!!")
            return RobotErrorType.RobotError_NotLogin

    def enter_tcp2canbus_mode(self):
        """
        * FUNCTION: enter_tcp2canbus_mode
        * DESCRIPTION: Notify the server to enter TCP2CANBUS transparent transmission mode
        * INPUTS:
        * OUTPUTS:
        * RETURNS: Successful return: RobotError.RobotError_SUCC
        * Failure return: other
        * NOTES:
        """
        self.check_event()
        if self.rshd >= 0 and self.connected:
            return libpyauboi5.enter_tcp2canbus_mode(self.rshd)
        else:
            logger.warn("RSHD uninitialized or not login!!!")
            return RobotErrorType.RobotError_NotLogin

    def leave_tcp2canbus_mode(self):
        """
        * FUNCTION: leave_tcp2canbus_mode
        * DESCRIPTION: Notify the server to exit the TCP2CANBUS transparent transmission mode
        * INPUTS:
        * OUTPUTS:
        * RETURNS: Successful return: RobotError.RobotError_SUCC
        * Failure return: other
        * NOTES:
        """
        self.check_event()
        if self.rshd >= 0 and self.connected:
            return libpyauboi5.leave_tcp2canbus_mode(self.rshd)
        else:
            logger.warn("RSHD uninitialized or not login!!!")
            return RobotErrorType.RobotError_NotLogin

    def set_waypoint_to_canbus(self, joint_radian=(0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000)):
        """
        * FUNCTION: set_waypoint_to_canbus
        * DESCRIPTION: Transparently transmit movement waypoints to CANBUS
        * INPUTS: joint_radian: joint angle of six joints, unit (rad)
        * OUTPUTS:
        * RETURNS: Successful return: RobotError.RobotError_SUCC
        * Failure return: other
        * NOTES:
        """
        self.check_event()
        if self.rshd >= 0 and self.connected:
            return libpyauboi5.set_waypoint_to_canbus(self.rshd, joint_radian)
        else:
            logger.warn("RSHD uninitialized or not login!!!")
            return RobotErrorType.RobotError_NotLogin

    def remove_all_waypoint(self):
        """
        * FUNCTION: remove_all_waypoint
        * DESCRIPTION: Clear all global waypoints that have been set
        * INPUTS:
        * OUTPUTS:
        * RETURNS: Successful return: RobotError.RobotError_SUCC
        * Failure return: other
        * NOTES:
        """
        self.check_event()
        if self.rshd >= 0 and self.connected:
            return libpyauboi5.remove_all_waypoint(self.rshd)
        else:
            logger.warn("RSHD uninitialized or not login!!!")
            return RobotErrorType.RobotError_NotLogin

    def add_waypoint(self, joint_radian=(0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000)):
        """
        * FUNCTION: add_waypoint
        * DESCRIPTION: Add global waypoints for trajectory movement
        * INPUTS: joint_radian: joint angle of six joints, unit (rad)
        * OUTPUTS:
        * RETURNS: Successful return: RobotError.RobotError_SUCC
        * Failure return: other
        * NOTES:
        """
        self.check_event()
        if self.rshd >= 0 and self.connected:
            return libpyauboi5.add_waypoint(self.rshd, joint_radian)
        else:
            logger.warn("RSHD uninitialized or not login!!!")
            return RobotErrorType.RobotError_NotLogin

    def set_blend_radius(self, blend_radius=0.01):
        """
        * FUNCTION: set_blend_radius
        * DESCRIPTION: Set the blend radius
        * INPUTS: blend_radius: blending radius, unit (m)
        * OUTPUTS:
        * RETURNS: Successful return: RobotError.RobotError_SUCC
        * Failure return: other
        * NOTES:
        """
        self.check_event()
        if self.rshd >= 0 and self.connected:
            if 0.01 >= blend_radius <= 0.05:
                return libpyauboi5.set_blend_radius(self.rshd, blend_radius)
            else:
                logger.warn("blend radius value range must be 0.01~0.05")
                return RobotErrorType.RobotError_ERROR_ARGS
        else:
            logger.warn("RSHD uninitialized or not login!!!")
            return RobotErrorType.RobotError_NotLogin

    def set_circular_loop_times(self, circular_count=1):
        """
        * FUNCTION: set_circular_loop_times
        * DESCRIPTION: Set the number of circle movement
        * INPUTS: circular_count: the number of circle movement
        * OUTPUTS:
        * RETURNS: Successful return: RobotError.RobotError_SUCC
        * Failure return: other
        *
        * NOTES: When circular_count is greater than 0, the robotic arm performs circular movement circular_count times
        * When circular_count is equal to 0, the robotic arm moves on a circular arc trajectory
        """
        self.check_event()
        if self.rshd >= 0 and self.connected:
            return libpyauboi5.set_circular_loop_times(self.rshd, circular_count)
        else:
            logger.warn("RSHD uninitialized or not login!!!")
            return RobotErrorType.RobotError_NotLogin

    def set_user_coord(self, user_coord):
        """
        * FUNCTION: set_user_coord
        * DESCRIPTION: Set user coordinate system
        * INPUTS: user_coord: user coordinate system
        * user_coord = {'coord_type': RobotCoordType.Robot_World_Coordinate,
        *'calibrate_method': RobotCoordCalMethod.CoordCalMethod_xOy,
        *'calibrate_points':
        * {"point1": (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        * "point2": (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        * "point3": (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)},
        *'tool_desc':
        * {"pos": (0.0, 0.0, 0.0),
        * "ori": (1.0, 0.0, 0.0, 0.0)}
        *}
        * OUTPUTS:
        * RETURNS: Successful return: RobotError.RobotError_SUCC
        * Failure return: other
        * NOTES:
        """
        self.check_event()
        if self.rshd >= 0 and self.connected:
            return libpyauboi5.set_user_coord(self.rshd, user_coord)
        else:
            logger.warn("RSHD uninitialized or not login!!!")
            return RobotErrorType.RobotError_NotLogin

    def set_base_coord(self):
        """
        * FUNCTION: set_base_coord
        * DESCRIPTION: Set the base coordinate system
        * INPUTS:
        * OUTPUTS:
        * RETURNS: Successful return: RobotError.RobotError_SUCC
        * Failure return: other
        * NOTES:
        """
        self.check_event()
        if self.rshd >= 0 and self.connected:
            return libpyauboi5.set_base_coord(self.rshd)
        else:
            logger.warn("RSHD uninitialized or not login!!!")
            return RobotErrorType.RobotError_NotLogin

    def check_user_coord(self, user_coord):
        """
        * FUNCTION: check_user_coord
        * DESCRIPTION: Check whether the user coordinate system parameter setting is reasonable
        * INPUTS: user_coord: user coordinate system
        * user_coord = {'coord_type': 2,
        *'calibrate_method': 0,
        *'calibrate_points':
        * {"point1": (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        * "point2": (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        * "point3": (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)},
        *'tool_desc':
        * {"pos": (0.0, 0.0, 0.0),
        * "ori": (1.0, 0.0, 0.0, 0.0)}
        *}
        * OUTPUTS:
        * RETURNS: reasonable return: RobotError.RobotError_SUCC
        * Unreasonable return: other
        * NOTES:
        """
        return libpyauboi5.check_user_coord(self.rshd, user_coord)

    def set_relative_offset_on_base(self, relative_pos, relative_ori):
        """
        * FUNCTION: set_relative_offset_on_base
        * DESCRIPTION: Set the movement offset based on the base standard system
        * INPUTS: relative_pos=(x, y, z) relative displacement, unit (m)
        * relative_ori=(w,x,y,z) relative attitude
        * OUTPUTS:
        * RETURNS: Successful return: RobotError.RobotError_SUCC
        * Failure return: other
        * NOTES:
        """
        self.check_event()
        if self.rshd >= 0 and self.connected:
            return libpyauboi5.set_relative_offset_on_base(self.rshd, relative_pos, relative_ori)
            
        else:
            logger.warn("RSHD uninitialized or not login!!!")
            return RobotErrorType.RobotError_NotLogin

    def set_relative_offset_on_user(self, relative_pos, relative_ori, user_coord):
        """
        * FUNCTION: set_relative_offset_on_user
        * DESCRIPTION: Set the movement offset based on the user's standard system
        * INPUTS: relative_pos=(x, y, z) relative displacement, unit (m)
        * relative_ori=(w,x,y,z) target attitude
        * user_coord: user coordinate system
        * user_coord = {'coord_type': 2,
        *'calibrate_method': 0,
        *'calibrate_points':
        * {"point1": (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        * "point2": (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        * "point3": (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)},
        *'tool_desc':
        * {"pos": (0.0, 0.0, 0.0),
        * "ori": (1.0, 0.0, 0.0, 0.0)}
        *}
        * OUTPUTS:
        * RETURNS: Successful return: RobotError.RobotError_SUCC
        * Failure return: other
        * NOTES:
        """
        self.check_event()
        if self.rshd >= 0 and self.connected:
            return libpyauboi5.set_relative_offset_on_user(self.rshd, relative_pos, relative_ori, user_coord)
        else:
            logger.warn("RSHD uninitialized or not login!!!")
            return RobotErrorType.RobotError_NotLogin

    def set_no_arrival_ahead(self):
        """
        * FUNCTION: set_no_arrival_ahead
        * DESCRIPTION: Cancel early setting
        * INPUTS:
        *
        * OUTPUTS:
        * RETURNS: Successful return: RobotError.RobotError_SUCC
        * Failure return: other
        * NOTES:
        """
        self.check_event()
        if self.rshd >= 0 and self.connected:
            result = libpyauboi5.set_no_arrival_ahead(self.rshd)
            if result != 0:
                self.raise_error(RobotErrorType.RobotError_Move, result, "set no arrival ahead error")
            else:
                return RobotErrorType.RobotError_SUCC
        else:
            logger.warn("RSHD uninitialized or not login!!!")
            return RobotErrorType.RobotError_NotLogin

    def set_arrival_ahead_distance(self, distance=0.0):
        """
        * FUNCTION: set_arrival_ahead_distance
        * DESCRIPTION: Set the advance arrival distance in distance mode
        * INPUTS: distance unit (m)
        *
        * OUTPUTS:
        * RETURNS: Successful return: RobotError.RobotError_SUCC
        * Failure return: other
        * NOTES:
        """
        self.check_event()
        if self.rshd >= 0 and self.connected:
            result = libpyauboi5.set_arrival_ahead_distance(self.rshd, distance)
            if result != 0:
                self.raise_error(RobotErrorType.RobotError_Move, result, "set arrival ahead distance error")
            else:
                return RobotErrorType.RobotError_SUCC
        else:
            logger.warn("RSHD uninitialized or not login!!!")
            return RobotErrorType.RobotError_NotLogin

    def set_arrival_ahead_time(self, sec=0.0):
        """
        * FUNCTION: set_arrival_ahead_time
        * DESCRIPTION: Set the arrival time in advance in time mode
        * INPUTS: sec advance arrival time 　 unit (seconds)
        *
        * OUTPUTS:
        * RETURNS: Successful return: RobotError.RobotError_SUCC
        * Failure return: other
        * NOTES:
        """
        self.check_event()
        if self.rshd >= 0 and self.connected:
            result = libpyauboi5.set_arrival_ahead_time(self.rshd, sec)
            if result != 0:
                self.raise_error(RobotErrorType.RobotError_Move, result, "set arrival ahead time error")
            else:
                return RobotErrorType.RobotError_SUCC
        else:
            logger.warn("RSHD uninitialized or not login!!!")
            return RobotErrorType.RobotError_NotLogin

    def set_arrival_ahead_blend(self, distance=0.0):
        """
        * FUNCTION: set_arrival_ahead_blend
        * DESCRIPTION: Set the blending radius distance in distance mode
        * INPUTS: blend radius unit (m)
        *
        * OUTPUTS:
        * RETURNS: Successful return: RobotError.RobotError_SUCC
        * Failure return: other
        * NOTES:
        """
        self.check_event()
        if self.rshd >= 0 and self.connected:
            result = libpyauboi5.set_arrival_ahead_blend(self.rshd, distance)
            if result != 0:
                self.raise_error(RobotErrorType.RobotError_Move, result, "set arrival ahead blend error")
            else:
                return RobotErrorType.RobotError_SUCC
        else:
            logger.warn("RSHD uninitialized or not login!!!")
            return RobotErrorType.RobotError_NotLogin

    def move_track(self, track):
        """
        * FUNCTION: move_track
        * DESCRIPTION: trajectory movement
        * INPUTS: track type, including the following:
        * Arc motion RobotMoveTrackType.ARC_CIR
        * RobotMoveTrackType.CARTESIAN_MOVEP
        *
        * OUTPUTS:
        * RETURNS: Successful return: RobotError.RobotError_SUCC
        * Failure return: other
        * NOTES:
        """
        self.check_event()
        if self.rshd >= 0 and self.connected:
            result = libpyauboi5.move_track(self.rshd, track)
            if result != 0:
                self.raise_error(RobotErrorType.RobotError_Move, result, "move error")
            else:
                return RobotErrorType.RobotError_SUCC
        else:
            logger.warn("RSHD uninitialized or not login!!!")
            return RobotErrorType.RobotError_NotLogin

    def forward_kin(self, joint_radian=(0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000)):
        """
        * FUNCTION: forward_kin
        * DESCRIPTION: positive solution
        * INPUTS: joint_radian: joint angle of six joints, unit (rad)
        * OUTPUTS:
        * RETURNS: Successful return: the result of the correct solution of the joint, the result is detailed in NOTES
        * Failure return: None
        *
        * NOTES: Six joint angles {'joint': [1.0, 1.0, 1.0, 1.0, 1.0, 1.0],
        * Position'pos': [-0.06403157614989634, -0.4185973810159096, 0.816883228463401],
        * Attitude'ori': [-0.11863209307193756, 0.3820514380931854, 0.0, 0.9164950251579285]}
        """
        if self.rshd >= 0:
            return libpyauboi5.forward_kin(self.rshd, joint_radian)
        else:
            logger.warn("RSHD uninitialized or not login!!!")
            return None

    def inverse_kin(self, joint_radian=(0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000),
                    pos=(0.0, 0.0, 0.0), ori=(1.0, 0.0, 0.0, 0.0)):
        """
        * FUNCTION: forward_kin
        * DESCRIPTION: inverse solution
        * INPUTS: joint_radian: the joint angle of the six joints at the starting point, unit (rad)
        * pos position (x, y, z) unit (m)
        * ori pose (w, x, y, z)
        * OUTPUTS:
        * RETURNS: Successful return: the result of the correct solution of the joint, the result is detailed in NOTES
        * Failure return: None
        *
        * NOTES: Six joint angles {'joint': [1.0, 1.0, 1.0, 1.0, 1.0, 1.0],
        * Position'pos': [-0.06403157614989634, -0.4185973810159096, 0.816883228463401],
        * Attitude'ori': [-0.11863209307193756, 0.3820514380931854, 0.0, 0.9164950251579285]}
        """
        if self.rshd >= 0:
            return libpyauboi5.inverse_kin(self.rshd, joint_radian, pos, ori)
        else:
            logger.warn("RSHD uninitialized or not login!!!")
            return None

    def base_to_user(self, pos, ori, user_coord, user_tool):
        """
        * FUNCTION: base_to_user
        * DESCRIPTION: User coordinate system to base coordinate system
        * INPUTS: pos: position (x, y, z) unit under the base mark system (m)
        * ori: the posture under the pedestal standard system (w, x, y, z)
        * user_coord: user coordinate system
        * user_coord = {'coord_type': 2,
        *'calibrate_method': 0,
        *'calibrate_points':
        * {"point1": (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        * "point2": (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        * "point3": (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)},
        *'tool_desc':
        * {"pos": (0.0, 0.0, 0.0),
        * "ori": (1.0, 0.0, 0.0, 0.0)}
        *}
        * user_tool user tool description
        * user_tool={"pos": (x, y, z), "ori": (w, x, y, z)}
        * OUTPUTS:
        * RETURNS: Successful return: return position and posture {"pos": (x, y, z), "ori": (w, x, y, z)}
        * Failure return: None
        *
        * NOTES:
        """
        return libpyauboi5.base_to_user(self.rshd, pos, ori, user_coord, user_tool)

    def user_to_base(self, pos, ori, user_coord, user_tool):
        """
        * FUNCTION: user_to_base
        * DESCRIPTION: User coordinate system to base coordinate system
        * INPUTS: pos: position (x, y, z) under the user standard system unit (m)
        * ori: the posture under the user's standard system (w, x, y, z)
        * user_coord: user coordinate system
        * user_coord = {'coord_type': 2,
        *'calibrate_method': 0,
        *'calibrate_points':
        * {"point1": (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        * "point2": (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        * "point3": (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)},
        *'tool_desc':
        * {"pos": (0.0, 0.0, 0.0),
        * "ori": (1.0, 0.0, 0.0, 0.0)}
        *}
        * user_tool user tool description
        * user_tool={"pos": (x, y, z), "ori": (w, x, y, z)}
        * OUTPUTS:
        * RETURNS: Successful return: return position and posture {"pos": (x, y, z), "ori": (w, x, y, z)}
        * Failure return: None
        *
        * NOTES:
        """
        return libpyauboi5.user_to_base(self.rshd, pos, ori, user_coord, user_tool)

    def base_to_base_additional_tool(self, flange_pos, flange_ori, user_tool):
        """
        * FUNCTION: base_to_base_additional_tool
        * DESCRIPTION: Turn the base coordinate system to the base mark to get the position and posture of the end point of the tool
        * INPUTS: pos: based on the flange center position information (x, y, z) unit (m) of the base standard system
        * ori: Attitude information based on the base standard system (w, x, y, z)
        * user_tool user tool description
        * user_tool={"pos": (x, y, z), "ori": (w, x, y, z)}
        * OUTPUTS:
        * RETURNS: Successful return: Return the tool end position and posture information based on the base standard system {"pos": (x, y, z), "ori": (w, x, y, z)}
        * Failure return: None
        *
        * NOTES:
        """
        return libpyauboi5.base_to_base_additional_tool(self.rshd, flange_pos, flange_ori, user_tool)

    def rpy_to_quaternion(self, rpy):
        """
        * FUNCTION: rpy_to_quaternion
        * DESCRIPTION: Euler angle to quaternion
        * INPUTS: rpy: Euler angle (rx, ry, rz), unit (m)
        * OUTPUTS:
        * RETURNS: Successful return: quaternion result, the result is detailed in NOTES
        * Failure return: None
        *
        * NOTES: Four elements (w, x, y, z)
        """
        if self.rshd >= 0:
            return libpyauboi5.rpy_to_quaternion(self.rshd, rpy)
        else:
            logger.warn("RSHD uninitialized !!!")
            return None

    def quaternion_to_rpy(self, ori):
        """
        * FUNCTION: quaternion_to_rpy
        * DESCRIPTION: quaternion to Euler angle
        * INPUTS: Quaternion (w, x, y, z)
        * OUTPUTS:
        * RETURNS: Successful return: Euler angle result, see NOTES for details
        * Failure return: None
        *
        * NOTES: rpy: Euler angle (rx, ry, rz), unit (m)
        """
        if self.rshd >= 0:
            return libpyauboi5.quaternion_to_rpy(self.rshd, ori)
        else:
            logger.warn("RSHD uninitialized !!!")
            return None

    def set_tool_end_param(self, tool_end_param):
        """
        * FUNCTION: set_tool_end_param
        * DESCRIPTION: Set end tool parameters
        * INPUTS: End tool parameters: tool_end_param={"pos": (x, y, z), "ori": (w, x, y, z)}
        * OUTPUTS:
        * RETURNS: Successful return: RobotError.RobotError_SUCC
        * Failure return: other
        *
        * NOTES:
        """
        self.check_event()
        if self.rshd >= 0 and self.connected:
            return libpyauboi5.set_tool_end_param(self.rshd, tool_end_param)
        else:
            logger.warn("RSHD uninitialized or not login!!!")
            return None

    def set_none_tool_dynamics_param(self):
        """
        * FUNCTION: set_none_tool_dynamics_param
        * DESCRIPTION: Set kinetic parameters without tools
        * INPUTS:
        * OUTPUTS:
        * RETURNS: Successful return: RobotError.RobotError_SUCC
        * Failure return: other
        *
        * NOTES:
        """
        self.check_event()
        if self.rshd >= 0 and self.connected:
            return libpyauboi5.set_none_tool_dynamics_param(self.rshd)
        else:
            logger.warn("RSHD uninitialized or not login!!!")
            return None

    def set_tool_dynamics_param(self, tool_dynamics):
        """
        * FUNCTION: set_tool_end_param
        * DESCRIPTION: Set the kinetic parameters of the tool
        * INPUTS: tool_dynamics: kinematics parameters
        * tool_dynamics = position, unit (m): {"position": (0.0, 0.0, 0.0),
        * Payload, unit (kg): "payload": 1.0,
        * Inertia: "inertia": (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)}
        * OUTPUTS:
        * RETURNS: Successful return: RobotError.RobotError_SUCC
        * Failure return: other
        *
        * NOTES:
        """
        self.check_event()
        if self.rshd >= 0 and self.connected:
            return libpyauboi5.set_tool_dynamics_param(self.rshd, tool_dynamics)
        else:
            logger.warn("RSHD uninitialized or not login!!!")
            return None

    def get_tool_dynamics_param(self):
        """
        * FUNCTION: get_tool_dynamics_param
        * DESCRIPTION: Get end tool parameters
        * INPUTS:
        * OUTPUTS:
        * RETURNS: Successful return: Kinematics parameters
        * tool_dynamics = position, unit (m): {"position": (0.0, 0.0, 0.0),
        * Payload, unit (kg): "payload": 1.0,
        * Inertia: "inertia": (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)}
        *
        * Failure return: None
        *
        * NOTES:
        """
        self.check_event()
        if self.rshd >= 0 and self.connected:
            return libpyauboi5.get_tool_dynamics_param(self.rshd)
        else:
            logger.warn("RSHD uninitialized or not login!!!")
            return None

    def set_none_tool_kinematics_param(self):
        """
        * FUNCTION: set_none_tool_kinematics_param
        * DESCRIPTION: Set kinematic parameters without tools　
        * INPUTS:
        * OUTPUTS:
        * RETURNS: Successful return: RobotError.RobotError_SUCC
        * Failure return: other
        *
        * NOTES:
        """
        self.check_event()
        if self.rshd >= 0 and self.connected:
            return libpyauboi5.set_none_tool_kinematics_param(self.rshd)
        else:
            logger.warn("RSHD uninitialized or not login!!!")
            return None

    def set_tool_kinematics_param(self, tool_end_param):
        """
        * FUNCTION: set_tool_kinematics_param
        * DESCRIPTION: Set the kinematic parameters of the tool　
        * INPUTS: End tool parameters: tool_end_param={"pos": (x, y, z), "ori": (w, x, y, z)}
        * OUTPUTS:
        * RETURNS: Successful return: RobotError.RobotError_SUCC
        * Failure return: other
        *
        * NOTES:
        """
        self.check_event()
        if self.rshd >= 0 and self.connected:
            return libpyauboi5.set_tool_kinematics_param(self.rshd, tool_end_param)
        else:
            logger.warn("RSHD uninitialized or not login!!!")
            return None

    def get_tool_kinematics_param(self):
        """
        * FUNCTION: set_tool_kinematics_param
        * DESCRIPTION: Set the kinematic parameters of the tool　
        * INPUTS:
        * OUTPUTS:
        * RETURNS: Successful return: the kinematics parameters of the tool
        * tool_end_param={"pos": (x, y, z), "ori": (w, x, y, z)}
        *
        * Failure return: None
        * NOTES:
        """
        self.check_event()
        if self.rshd >= 0 and self.connected:
            return libpyauboi5.get_tool_kinematics_param(self.rshd)
        else:
            logger.warn("RSHD uninitialized or not login!!!")
            return None

    def move_stop(self):
        """
        * FUNCTION: move_stop
        * DESCRIPTION: Stop the robot arm movement
        * INPUTS:
        * OUTPUTS:
        * RETURNS: Successful return: RobotError.RobotError_SUCC
        * Failure return: other
        * NOTES:
        """
        self.check_event()
        if self.rshd >= 0 and self.connected:
            return libpyauboi5.move_stop(self.rshd)
        else:
            logger.warn("RSHD uninitialized or not login!!!")
            return RobotErrorType.RobotError_NotLogin

    def move_pause(self):
        """
        * FUNCTION: move_pause
        * DESCRIPTION: Pause robotic arm movement
        * INPUTS:
        * OUTPUTS:
        * RETURNS: Successful return: RobotError.RobotError_SUCC
        * Failure return: other
        * NOTES:
        """
        self.check_event()
        if self.rshd >= 0 and self.connected:
            return libpyauboi5.move_pause(self.rshd)
        else:
            logger.warn("RSHD uninitialized or not login!!!")
            return RobotErrorType.RobotError_NotLogin

    def move_continue(self):
        """
        * FUNCTION: move_continue
        * DESCRIPTION: Resume the movement of the robotic arm after a pause
        * INPUTS:
        * OUTPUTS:
        * RETURNS: Successful return: RobotError.RobotError_SUCC
        * Failure return: other
        * NOTES:
        """
        self.check_event()
        if self.rshd >= 0 and self.connected:
            return libpyauboi5.move_continue(self.rshd)
        else:
            logger.warn("RSHD uninitialized or not login!!!")
            return RobotErrorType.RobotError_NotLogin

    def collision_recover(self):
        """
        * FUNCTION: collision_recover
        * DESCRIPTION: Recovery after collision
        * INPUTS:
        * OUTPUTS:
        * RETURNS: Successful return: RobotError.RobotError_SUCC
        * Failure return: other
        * NOTES:
        """
        self.check_event()
        if self.rshd >= 0 and self.connected:
            return libpyauboi5.collision_recover(self.rshd)
        else:
            logger.warn("RSHD uninitialized or not login!!!")
            return RobotErrorType.RobotError_NotLogin

    def get_robot_state(self):
        """
        * FUNCTION: get_robot_state
        * DESCRIPTION: Get the current status of the robotic arm
        * INPUTS:
        * OUTPUTS:
        * RETURNS: Successful return: the current state of the robotic arm
        * The robot arm is currently stopped: RobotStatus.Stopped
        * The arm is currently running: RobotStatus.Running
        * The robotic arm is currently paused: RobotStatus.Paused
        * The robotic arm is currently restored: RobotStatus.Resumed
        *
        * Failure return: None
        * NOTES:
        """
        self.check_event()
        if self.rshd >= 0 and self.connected:
            return libpyauboi5.get_robot_state(self.rshd)
        else:
            logger.warn("RSHD uninitialized or not login!!!")
            return None

    def enter_reduce_mode(self):
        """
        * FUNCTION: enter_reduce_mode
        * DESCRIPTION: Set the robot arm movement into reduced mode
        * INPUTS:
        * OUTPUTS:
        * RETURNS: Successful return: RobotError.RobotError_SUCC
        * Failure return: other
        * NOTES:
        """
        self.check_event()
        if self.rshd >= 0 and self.connected:
            return libpyauboi5.enter_reduce_mode(self.rshd)
        else:
            logger.warn("RSHD uninitialized or not login!!!")
            return RobotErrorType.RobotError_NotLogin

    def exit_reduce_mode(self):
        """
        * FUNCTION: exit_reduce_mode
        * DESCRIPTION: Set the robot arm movement to exit the reduced mode
        * INPUTS:
        * OUTPUTS:
        * RETURNS: Successful return: RobotError.RobotError_SUCC
        * Failure return: other
        * NOTES:
        """
        self.check_event()
        if self.rshd >= 0 and self.connected:
            return libpyauboi5.exit_reduce_mode(self.rshd)
        else:
            logger.warn("RSHD uninitialized or not login!!!")
            return RobotErrorType.RobotError_NotLogin

    def project_startup(self):
        """
        * FUNCTION: project_startup
        * DESCRIPTION: Notify the robotic arm project to start, and the server starts to detect safety IO at the same time
        * INPUTS:
        * OUTPUTS:
        * RETURNS: Successful return: RobotError.RobotError_SUCC
        * Failure return: other
        * NOTES:
        """
        self.check_event()
        if self.rshd >= 0 and self.connected:
            return libpyauboi5.project_startup(self.rshd)
        else:
            logger.warn("RSHD uninitialized or not login!!!")
            return RobotErrorType.RobotError_NotLogin

    def rs_project_stop(self):
        """
        * FUNCTION: rs_project_stop
        * DESCRIPTION: Notify the robotic arm project to stop and the server to stop detecting safety IO
        * INPUTS:
        * OUTPUTS:
        * RETURNS: Successful return: RobotError.RobotError_SUCC
        * Failure return: other
        * NOTES:
        """
        self.check_event()
        if self.rshd >= 0 and self.connected:
            return libpyauboi5.rs_project_stop(self.rshd)
        else:
            logger.warn("RSHD uninitialized or not login!!!")
            return RobotErrorType.RobotError_NotLogin

    def set_work_mode(self, mode=0):
        """
        * FUNCTION: set_work_mode
        * DESCRIPTION: Set the working mode of the robotic arm server
        * INPUTS: mode: server working mode
        * Robotic arm simulation mode: RobotRunningMode.RobotModeSimulator
        * Robot real mode: RobotRunningMode.RobotModeReal
        * OUTPUTS:
        * RETURNS: Successful return: RobotError.RobotError_SUCC
        * Failure return: other
        * NOTES:
        """
        self.check_event()
        if self.rshd >= 0 and self.connected:
            return libpyauboi5.set_work_mode(self.rshd, mode)
        else:
            logger.warn("RSHD uninitialized or not login!!!")
            return RobotErrorType.RobotError_LOGIN_FAILED

    def get_work_mode(self):
        """
        * FUNCTION: set_work_mode
        * DESCRIPTION: Get the current working mode of the robotic arm server
        * INPUTS: mode: server working mode
        * Robotic arm simulation mode: RobotRunningMode.RobotModeSimulator
        * Robot real mode: RobotRunningMode.RobotModeReal
        * OUTPUTS:
        * RETURNS: Successful return: server working mode
        * Robotic arm simulation mode: RobotRunningMode.RobotModeSimulator
        * Robot real mode: RobotRunningMode.RobotModeReal
        *
        * Failure return: None
        * NOTES:
        """
        self.check_event()
        if self.rshd >= 0 and self.connected:
            return libpyauboi5.get_work_mode(self.rshd)
        else:
            logger.warn("RSHD uninitialized or not login!!!")
            return None

    def set_collision_class(self, grade=6):
        """
        * FUNCTION: set_collision_class
        * DESCRIPTION: Set the collision level of the robotic arm
        * INPUTS: grade collision grade: collision grade range (0～10)
        * OUTPUTS:
        * RETURNS: Successful return: RobotError.RobotError_SUCC
        * Failure return: other
        * NOTES:
        """
        self.check_event()
        if self.rshd >= 0 and self.connected:
            return libpyauboi5.set_collision_class(self.rshd, grade)
        else:
            logger.warn("RSHD uninitialized or not login!!!")
            return RobotErrorType.RobotError_LOGIN_FAILED

    def is_have_real_robot(self):
        """
        * FUNCTION: is_have_real_robot
        * DESCRIPTION: Get whether the real robot arm is currently connected
        * INPUTS:
        * OUTPUTS:
        * RETURNS: successful return: 1: exists 0: does not exist
        * Failure return: other
        * NOTES:
        """
        self.check_event()
        if self.rshd >= 0 and self.connected:
            return libpyauboi5.is_have_real_robot(self.rshd)
        else:
            logger.warn("RSHD uninitialized or not login!!!")
            return None

    def is_online_mode(self):
        """
        * FUNCTION: is_online_mode
        * DESCRIPTION: Whether the current robotic arm is running in online mode
        * INPUTS:
        * OUTPUTS:
        * RETURNS: Successful return: 1: at 0: not at
        * Failure return: other
        * NOTES:
        """
        self.check_event()
        if self.rshd >= 0 and self.connected:
            return libpyauboi5.is_online_mode(self.rshd)
        else:
            logger.warn("RSHD uninitialized or not login!!!")
            return None

    def is_online_master_mode(self):
        """
        * FUNCTION: is_online_master_mode
        * DESCRIPTION: Whether the current robotic arm is running in online main mode
        * INPUTS:
        * OUTPUTS:
        * RETURNS: Successful return: 1: Master mode 0: Slave mode
        * Failure return: other
        * NOTES:
        """
        self.check_event()
        if self.rshd >= 0 and self.connected:
            return libpyauboi5.is_online_master_mode(self.rshd)
        else:
            logger.warn("RSHD uninitialized or not login!!!")
            return None

    def get_joint_status(self):
        """
        * FUNCTION: get_joint_status
        * DESCRIPTION: Get the current status information of the robotic arm
        * INPUTS:
        * OUTPUTS:
        * RETURNS: successful return: return to the six joint states, including: current, voltage, temperature
        * {'joint1': {'current': current (mA),'voltage': voltage (volt),'temperature': temperature (degree Celsius)},
        *'joint2': {'current': 0,'voltage': 0.0,'temperature': 0},
        *'joint3': {'current': 0,'voltage': 0.0,'temperature': 0},
        *'joint4': {'current': 0,'voltage': 0.0,'temperature': 0},
        *'joint5': {'current': 0,'voltage': 0.0,'temperature': 0},
        *'joint6': {'current': 0,'voltage': 0.0,'temperature': 0}}
        *
        * Failure return: None
        * NOTES:
        """
        self.check_event()
        if self.rshd >= 0 and self.connected:
            return libpyauboi5.get_joint_status(self.rshd)
        else:
            logger.warn("RSHD uninitialized or not login!!!")
            return None

    # zhar2_rem
    def get_current_waypoint(self):
        """
        * FUNCTION: get_current_waypoint
        * DESCRIPTION: Get the current position information of the robotic arm
        * INPUTS: grade collision grade: collision grade range (0～10)
        * OUTPUTS:
        * RETURNS: Successful return: joint position information, the result is detailed in NOTES
        * Failure return: None
        *
        * NOTES: Six joint angles {'joint': [1.0, 1.0, 1.0, 1.0, 1.0, 1.0],
        * Position'pos': [-0.06403157614989634, -0.4185973810159096, 0.816883228463401],
        * Attitude'ori': [-0.11863209307193756, 0.3820514380931854, 0.0, 0.9164950251579285]}
        """
        self.check_event()
        if self.rshd >= 0 and self.connected:
            return libpyauboi5.get_current_waypoint(self.rshd)
        else:
            logger.warn("RSHD uninitialized or not login!!!")
            return None

    def get_board_io_config(self, io_type=RobotIOType.User_DO):
        """
        * FUNCTION: get_board_io_config
        * DESCRIPTION:
        * INPUTS: io_type: IO type: RobotIOType
        * OUTPUTS:
        * RETURNS: Successful return: IO configuration
        * [{"id": ID
        * "name": "IO name"
        * "addr": IO address
        * "type": IO type
        * "value": IO current value},]
        *
        * Failure return: None
        * NOTES: For details about RobotIOType, please refer to class RobotIOType
        """
        self.check_event()
        if self.rshd >= 0 and self.connected:
            return libpyauboi5.get_board_io_config(self.rshd, io_type)
        else:
            logger.warn("RSHD uninitialized or not login!!!")
            return None

    def get_board_io_status(self, io_type, io_name):
        """
        * FUNCTION: get_board_io_status
        * DESCRIPTION: Get IO status
        * INPUTS: io_type: type
        * io_name: name RobotUserIoName.user_dx_xx
        * OUTPUTS:
        * RETURNS: Successful return: IO status double value (digital IO, return 0 or 1, analog IO return floating point number)
        * Failure return: None
        * NOTES:
        """
        self.check_event()
        if self.rshd >= 0 and self.connected:
            return libpyauboi5.get_board_io_status(self.rshd, io_type, io_name)
        else:
            logger.warn("RSHD uninitialized or not login!!!")
            return None

    def set_board_io_status(self, io_type, io_name, io_value):
        """
        * FUNCTION: set_board_io_status
        * DESCRIPTION: Set IO status
        * INPUTS: io_type: type
        * io_name: name RobotUserIoName.user_dx_xx
        * io_value: state value (digital IO, returns 0 or 1, analog IO returns a floating point number)
        * OUTPUTS:
        * RETURNS: Successful return: RobotError.RobotError_SUCC
        * Failure return: other
        * NOTES:
        """
        #self.check_event()
        if self.rshd >= 0 and self.connected:
            return libpyauboi5.set_board_io_status(self.rshd, io_type, io_name, io_value)
        else:
            logger.warn("RSHD uninitialized or not login!!!")
            return RobotErrorType.RobotError_LOGIN_FAILED

    def set_tool_power_type(self, power_type=RobotToolPowerType.OUT_0V):
        """
        * FUNCTION: set_tool_power_type
        * DESCRIPTION: Set the power supply type of the tool
        * INPUTS: power_type: power type
        * RobotToolPowerType.OUT_0V
        * RobotToolPowerType.OUT_12V
        * RobotToolPowerType.OUT_24V
        * OUTPUTS:
        * RETURNS: Successful return: RobotError.RobotError_SUCC
        * Failure return: other
        * NOTES:
        """
        self.check_event()
        if self.rshd >= 0 and self.connected:
            return libpyauboi5.set_tool_power_type(self.rshd, power_type)
        else:
            logger.warn("RSHD uninitialized or not login!!!")
            return RobotErrorType.RobotError_LOGIN_FAILED

    def get_tool_power_type(self):
        """
        * FUNCTION: get_tool_power_type
        * DESCRIPTION: Get the power supply type of the tool
        * INPUTS: power_type: power type

        * OUTPUTS:
        * RETURNS: Successful return: Power source type, including the following:
        * RobotToolPowerType.OUT_0V
        * RobotToolPowerType.OUT_12V
        * RobotToolPowerType.OUT_24V
        *
        * Failure return: None
        * NOTES:
        """
        self.check_event()
        if self.rshd >= 0 and self.connected:
            return libpyauboi5.get_tool_power_type(self.rshd)
        else:
            logger.warn("RSHD uninitialized or not login!!!")
            return None

    def set_tool_io_type(self, io_addr=RobotToolIoAddr.TOOL_DIGITAL_IO_0,
                         io_type=RobotToolDigitalIoDir.IO_OUT):
        """
        * FUNCTION: set_tool_io_type
        * DESCRIPTION: Set the tool-side digital IO type
        * INPUTS: io_addr: Tool end IO address, see class RobotToolIoAddr for details
        * io_type: Tool-side IO type, see class RobotToolDigitalIoDir for details

        * OUTPUTS:
        * RETURNS: Successful return: IO type, including the following:
        * RobotToolDigitalIoDir.IO_IN
        * RobotToolDigitalIoDir.IO_OUT
        *
        * Failure return: None
        * NOTES:
        """
        self.check_event()
        if self.rshd >= 0 and self.connected:
            return libpyauboi5.set_tool_io_type(self.rshd, io_addr, io_type)
        else:
            logger.warn("RSHD uninitialized or not login!!!")
            return RobotErrorType.RobotError_LOGIN_FAILED

    def get_tool_power_voltage(self):
        """
        * FUNCTION: get_tool_power_voltage
        * DESCRIPTION: Get the voltage value of the tool end
        * INPUTS:
        * OUTPUTS:
        * RETURNS: successful return: return voltage value, unit (volt)
        * Failure return: None
        * NOTES:
        """
        self.check_event()
        if self.rshd >= 0 and self.connected:
            return libpyauboi5.get_tool_power_voltage(self.rshd)
        else:
            logger.warn("RSHD uninitialized or not login!!!")
            return None

    def get_tool_io_status(self, io_name):
        """
        * FUNCTION: get_tool_io_status
        * DESCRIPTION: Get the IO status of the tool
        * INPUTS: io_name: IO name

        * OUTPUTS:
        * RETURNS: Successful return: Return to the tool-side IO status
        *
        * Failure return: None
        * NOTES:
        """
        self.check_event()
        if self.rshd >= 0 and self.connected:
            return libpyauboi5.get_tool_io_status(self.rshd, io_name)
        else:
            logger.warn("RSHD uninitialized or not login!!!")
            return None

    def set_tool_io_status(self, io_name, io_status):
        """
        * FUNCTION: set_tool_io_status
        * DESCRIPTION: Set the IO status of the tool
        * INPUTS: io_name: tool-side IO name
        * io_status: Tool-side IO status: value range (0 or 1)

        * OUTPUTS:
        * RETURNS: Successful return: RobotError.RobotError_SUCC
        * Failure return: other
        * NOTES:
        """
        self.check_event()
        if self.rshd >= 0 and self.connected:
            return libpyauboi5.set_tool_do_status(self.rshd, io_name, io_status)
        else:
            logger.warn("RSHD uninitialized or not login!!!")
            return RobotErrorType.RobotError_LOGIN_FAILED

    def startup_excit_traj_track(self, track_file='', track_type=0, subtype=0):
        """
        * FUNCTION: startup_excit_traj_track
        * DESCRIPTION: Notify the server to start the recognition track movement
        * INPUTS:
        * OUTPUTS:
        * RETURNS: Successful return: RobotError.RobotError_SUCC
        * Failure return: other
        * NOTES:
        """
        self.check_event()
        if self.rshd >= 0 and self.connected:
            return libpyauboi5.startup_excit_traj_track(self.rshd, track_file, track_type, subtype)
        else:
            logger.warn("RSHD uninitialized or not login!!!")
            return RobotErrorType.RobotError_NotLogin

    def get_dynidentify_results(self):
        """
        * FUNCTION: get_dynidentify_results
        * DESCRIPTION: Get the identification result
        * INPUTS:
        * OUTPUTS:
        * RETURNS: Successful return: identification result array
        * Failure return: None
        * NOTES:
        """
        self.check_event()
        if self.rshd >= 0 and self.connected:
            return libpyauboi5.get_dynidentify_results(self.rshd)
        else:
            logger.warn("RSHD uninitialized or not login!!!")
            return None

    def set_robot_event_callback(self, callback):
        """
        * FUNCTION: set_robot_event_callback
        * DESCRIPTION: Set the robot arm event callback function
        * INPUTS: callback: callback function name
        * OUTPUTS:
        * RETURNS: Successful return: RobotError.RobotError_SUCC
        * Failure return: other
        * NOTES:
        """
        if self.rshd >= 0 and self.connected:
            return libpyauboi5.set_robot_event_callback(self.rshd, callback)
        else:
            logger.warn("RSHD uninitialized or not login!!!")
            return RobotErrorType.RobotError_LOGIN_FAILED


# Test function
def test(test_count):
    # Initialize logger
    logger_init()

    # Start test
    logger.info("{0} test beginning...".format(Auboi5Robot.get_local_time()))

    # system initialization
    Auboi5Robot.initialize()

    # Create a robotic arm control class
    robot = Auboi5Robot()

    # Create context
    handle = robot.create_context()

    # Print context
    logger.info("robot.rshd={0}".format(handle))

    try:

        # Link server
        ip ='localhost'
        #ip = '192.168.199.200'

        port = 8899
        result = robot.connect(ip, port)

        if result != RobotErrorType.RobotError_SUCC:
            logger.info("connect server{0}:{1} failed.".format(ip, port))
        else:
            # # Power on again
            #robot.robot_shutdown()
            #
            # # Power-on
            robot.robot_startup()
            #
            # # Set collision level
            robot.set_collision_class(7)

            # Set the tool end power supply to 12v
            # robot.set_tool_power_type(RobotToolPowerType.OUT_12V)

            # Set tool end IO_0 as output
            #robot.set_tool_io_type(RobotToolIoAddr.TOOL_DIGITAL_IO_0, RobotToolDigitalIoDir.IO_OUT)

            # Get the current status of tool IO_0
            #tool_io_status = robot.get_tool_io_status(RobotToolIoName.tool_io_0)
            #logger.info("tool_io_0={0}".format(tool_io_status))

            # Set tool-side IO_0 status
            #robot.set_tool_io_status(RobotToolIoName.tool_io_0, 1)


            # Get control cabinet user DO
            #io_config = robot.get_board_io_config(RobotIOType.User_DO)

            # Output DO configuration
            #logger.info(io_config)

            # Is the current robotic arm running in online mode?
            #logger.info("robot online mode is {0}".format(robot.is_online_mode()))

            # Cycle test
            while test_count> 0:
                test_count -= 1

                joint_status = robot.get_joint_status()
                logger.info("joint_status={0}".format(joint_status))

                # Initialize the global configuration file
                robot.init_profile()

                # Set the maximum acceleration of the joint
                robot.set_joint_maxacc((1.5, 1.5, 1.5, 1.5, 1.5, 1.5))

                # Set the maximum acceleration of the joint
                robot.set_joint_maxvelc((1.5, 1.5, 1.5, 1.5, 1.5, 1.5))

                joint_radian = (0.541678, 0.225068, -0.948709, 0.397018, -1.570800, 0.541673)
                logger.info("move joint to {0}".format(joint_radian))

                robot.move_joint(joint_radian)

                # Get the maximum acceleration of the joint
                logger.info(robot.get_joint_maxacc())

                # Positive solution test
                fk_ret = robot.forward_kin((-0.000003, -0.127267, -1.321122, 0.376934, -1.570796, -0.000008))
                logger.info(fk_ret)

                # Inverse solution
                joint_radian = (0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000)
                ik_result = robot.inverse_kin(joint_radian, fk_ret['pos'], fk_ret['ori'])
                logger.info(ik_result)

                # 轴动1
                joint_radian = (0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000)
                logger.info("move joint to {0}".format(joint_radian))
                robot.move_joint(joint_radian)

                # 轴动2
                joint_radian = (0.541678, 0.225068, -0.948709, 0.397018, -1.570800, 0.541673)
                logger.info("move joint to {0}".format(joint_radian))
                robot.move_joint(joint_radian)

                # 轴动3
                joint_radian = (-0.000003, -0.127267, -1.321122, 0.376934, -1.570796, -0.000008)
                logger.info("move joint to {0}".format(joint_radian))
                robot.move_joint(joint_radian)

                # Set the maximum linear acceleration at the end of the robotic arm (m/s)
                robot.set_end_max_line_acc(0.5)

                # Get the maximum linear acceleration at the end of the robotic arm (m/s)
                robot.set_end_max_line_velc(0.2)

                # Clear all global waypoints that have been set
                robot.remove_all_waypoint()

                # Add global waypoint 1 for trajectory movement
                joint_radian = (-0.000003, -0.127267, -1.321122, 0.376934, -1.570796, -0.000008)
                robot.add_waypoint(joint_radian)

                # Add global waypoint 2 for trajectory movement
                joint_radian = (-0.211675, -0.325189, -1.466753, 0.429232, -1.570794, -0.211680)
                robot.add_waypoint(joint_radian)

                # Add global waypoint 3 for trajectory movement
                joint_radian = (-0.037186, -0.224307, -1.398285, 0.396819, -1.570796, -0.037191)
                robot.add_waypoint(joint_radian)

                # Set the number of circular motions
                robot.set_circular_loop_times(3)

                #Circular movement
                logger.info("move_track ARC_CIR")
                robot.move_track(RobotMoveTrackType.ARC_CIR)

                # Clear all global waypoints that have been set
                robot.remove_all_waypoint()

                # Robot arm axis moves back to 0 position
                joint_radian = (0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000)
                logger.info("move joint to {0}".format(joint_radian))
                robot.move_joint(joint_radian)

            # Disconnect server link
            robot.disconnect()

    except RobotError as e:
        logger.error("{0} robot Event:{1}".format(robot.get_local_time(), e))


    finally:
        # Disconnect server link
        if robot.connected:
            # Close the robotic arm
            robot.robot_shutdown()
            # Disconnect the robot arm link
            robot.disconnect()
        # Release library resources
        Auboi5Robot.uninitialize()
        logger.info("{0} test completed.".format(Auboi5Robot.get_local_time()))


def step_test():
    # Initialize logger
    logger_init()

    # Start test
    logger.info("{0} test beginning...".format(Auboi5Robot.get_local_time()))

    # system initialization
    Auboi5Robot.initialize()

    # Create a robotic arm control class
    robot = Auboi5Robot()

    # Create context
    handle = robot.create_context()

    # Print context
    logger.info("robot.rshd={0}".format(handle))

    try:

        # Link server
        ip ='localhost'
        port = 8899
        result = robot.connect(ip, port)

        if result != RobotErrorType.RobotError_SUCC:
            logger.info("connect server{0}:{1} failed.".format(ip, port))
        else:
            # Power on again
            robot.robot_shutdown()

            # Power-on
            robot.robot_startup()

            # Set collision level
            robot.set_collision_class(7)

            # # Initialize the global configuration file
            # robot.init_profile()
            #
            # # logger.info(robot.get_board_io_config(RobotIOType.User_DI))
            #
            # # Get current location
            # logger.info(robot.get_current_waypoint())
            #
            # joint_radian = (0, 0, 0, 0, 0, 0)
            # # Axis moves to initial position
            # robot.move_joint(joint_radian)
            #
            # # Move 0.1 mm along the Z axis
            # current_pos = robot.get_current_waypoint()
            #
            # current_pos['pos'][2] -= 0.001
            #
            # ik_result = robot.inverse_kin(current_pos['joint'], current_pos['pos'], current_pos['ori'])
            # logger.info(ik_result)
            #
            # # joint_radian = (0.541678, 0.225068, -0.948709, 0.397018, -1.570800, 0.541673)
            # # logger.info("move joint to {0}".format(joint_radian))
            # # robot.move_joint(joint_radian)
            #
            # robot.move_line(ik_result['joint'])

            # Disconnect server link
            robot.disconnect()

    except RobotError as e:
        logger.error("robot Event:{0}".format(e))

    finally:
        # Disconnect server link
        if robot.connected:
            # Disconnect the robot arm link
            robot.disconnect()
        # Release library resources
        Auboi5Robot.uninitialize()
        logger.info("{0} test completed.".format(Auboi5Robot.get_local_time()))


def excit_traj_track_test():
    # Initialize logger
    logger_init()

    # Start test
    logger.info("{0} test beginning...".format(Auboi5Robot.get_local_time()))

    # system initialization
    Auboi5Robot.initialize()

    # Create a robotic arm control class
    robot = Auboi5Robot()

    # Create context
    handle = robot.create_context()

    # Print context
    logger.info("robot.rshd={0}".format(handle))

    try:

        # Link server
        ip ='localhost'
        port = 8899
        result = robot.connect(ip, port)

        if result != RobotErrorType.RobotError_SUCC:
            logger.info("connect server{0}:{1} failed.".format(ip, port))
        else:

            # Power on again
            # robot.robot_shutdown()

            # Power-on
            # robot.robot_startup()

            # Set collision level
            # robot.set_collision_class(7)

            joint_radian = (0, 0, 0, 0, 0, 0)
            # The axis moves to the initial position
            robot.move_joint(joint_radian)

            logger.info("starup excit traj track....")

            # Start identification track
            #robot.startup_excit_traj_track("dynamics_exciting_trajectories/excitTraj1.offt", 1, 0)

            # Delay two seconds to wait for the identification result
            #time.sleep(5)

            # Get recognition results
            dynidentify_ret = robot.get_dynidentify_results()
            logger.info("dynidentify result={0}".format(dynidentify_ret))
            for i in range(0,54):
                dynidentify_ret[i] = dynidentify_ret[i]/1024.0
            logger.info("dynidentify result={0}".format(dynidentify_ret))

            # Disconnect server link
            robot.disconnect()

    except RobotError as e:
        logger.error("robot Event:{0}".format(e))


    finally:
        # Disconnect server link
        if robot.connected:
            # Disconnect the robot arm link
            robot.disconnect()
        # Release library resources
        Auboi5Robot.uninitialize()


def move_rotate_test():
    # Initialize logger
    logger_init()

    # Start test
    logger.info("{0} test beginning...".format(Auboi5Robot.get_local_time()))

    # system initialization
    Auboi5Robot.initialize()

    # Create a robotic arm control class
    robot = Auboi5Robot()

    # Create context
    handle = robot.create_context()

    # Print context
    logger.info("robot.rshd={0}".format(handle))

    try:

        # Link server
        ip ='localhost'
        port = 8899
        result = robot.connect(ip, port)

        if result != RobotErrorType.RobotError_SUCC:
            logger.info("connect server{0}:{1} failed.".format(ip, port))
        else:

            # Power on again
            # robot.robot_shutdown()

            # Power-on
            # robot.robot_startup()

            # Set collision level
            # robot.set_collision_class(7)

            # joint_radian = (1, 0, 0, 0, 0, 0)
            # # Axis moves to initial position
            # robot.move_joint(joint_radian)

            joint_radian = (0.541678, 0.225068, -0.948709, 0.397018, -1.570800, 0.541673)
            logger.info("move joint to {0}".format(joint_radian))
            robot.move_joint(joint_radian)

            # Get current location
            current_pos = robot.get_current_waypoint()

            # The vector of the tool shaft (relative to the flange, so you need to measure x, y, z. This test sample defaults to x=0, y=0, and the z axis is 0.1 meters)
            tool_pos_on_end = (0, 0, 0.10)

            # Tool posture (w, x, y, z relative to the flange, if you don’t know, fill in the following information by default)
            tool_ori_on_end = (1, 0, 0, 0)

            tool_desc = {"pos": tool_pos_on_end, "ori": tool_ori_on_end}

            # Get the position of the end point of the flange tool relative to the base coordinate system
            tool_pos_on_base = robot.base_to_base_additional_tool(current_pos['pos'],
                                                                  current_pos['ori'],
                                                                  tool_desc)

            logger.info("current_pos={0}".format(current_pos['pos'][0]))

            logger.info("tool_pos_on_base={0}".format(tool_pos_on_base['pos'][0]))

            # Talk about the translation of the tool axis vector to the base coordinate system (the direction of rotation conforms to the right-hand rule)
            rotate_axis = map(lambda a, b: a-b, tool_pos_on_base['pos'], current_pos['pos'])

            logger.info("rotate_axis={0}".format(rotate_axis))

            # The coordinate system uses the base coordinate system by default (fill in the following values ​​by default)
            user_coord = {'coord_type': RobotCoordType.Robot_Base_Coordinate,
                          'calibrate_method': 0,
                          'calibrate_points':
                              {"point1": (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
                               "point2": (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
                               "point3": (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)},
                          'tool_desc':
                              {"pos": (0.0, 0.0, 0.0),
                               "ori": (1.0, 0.0, 0.0, 0.0)}
                          }

            # Call the shaft rotation interface, the last parameter is the rotation angle (radians)
            robot.move_rotate(user_coord, rotate_axis, 1)

            # Disconnect server link
            robot.disconnect()

    except RobotError as e:
        logger.error("robot Event:{0}".format(e))

    finally:
        # Disconnect server link
        if robot.connected:
            # Disconnect the robot arm link
            robot.disconnect()
        # Release library resources
        Auboi5Robot.uninitialize()


def test_rsm():
    # Initialize logger
    logger_init()

    # Start test
    logger.info("{0} test beginning...".format(Auboi5Robot.get_local_time()))

    # system initialization
    Auboi5Robot.initialize()

    # Create a robotic arm control class
    robot = Auboi5Robot()

    # Create context
    handle = robot.create_context()

    # Print context
    logger.info("robot.rshd={0}".format(handle))

    try:

        # Link server
        #ip ='localhost'
        ip = '192.168.10.88'
        port = 8899
        result = robot.connect(ip, port)
        
        #robot.enable_robot_event()

        if result != RobotErrorType.RobotError_SUCC:
            logger.info("connect server{0}:{1} failed.".format(ip, port))
        else:

            # robot.move_pause()

            #joint_radian = (0, 0, 0, 0, 0, 0)
            # The axis moves to the initial position
            #robot.move_joint(joint_radian)

            while True:
                time.sleep(0.05)

                rel = robot.set_board_io_status(RobotIOType.User_DO, RobotUserIoName.user_do_02, 0)
                print(rel)
                print("++++++++++++++++++++++++++")
                #result = robot.get_board_io_status(RobotIOType.User_DO, RobotUserIoName.user_do_02)
                #print(result)
                # print("*********************************")

                time.sleep(2)
                # rel1 = robot.set_board_io_status(RobotIOType.User_DO, RobotUserIoName.user_do_02, 0)
                # print(rel1)
                # print("++++++++++++++++++++++++++")

            # Disconnect server link
            robot.disconnect()

    except RobotError as e:
        logger.error("robot Event:{0}".format(e))


    finally:
        # Disconnect server link
        if robot.connected:
            # Disconnect the robot arm link
            robot.disconnect()
        # Release library resources
        Auboi5Robot.uninitialize()


class GetRobotWaypointProcess(Process):
    def __init__(self):
        Process.__init__(self)
        self.isRunWaypoint = False
        self._waypoints = None


    def startMoveList(self, waypoints):
        if self.isRunWaypoint == True:
            return False
        else:
            self._waypoints = waypoints

    def run(self):
        # Initialize logger
        logger_init()

        # Start test
        logger.info("{0} test beginning...".format(Auboi5Robot.get_local_time()))

        # system initialization
        Auboi5Robot.initialize()

        # Create a robotic arm control class
        robot = Auboi5Robot()

        # Create context
        handle = robot.create_context()

        # Print context
        logger.info("robot.rshd={0}".format(handle))

        try:
            # Link server
            #ip ='localhost'
            ip = '192.168.65.131'
            port = 8899
            result = robot.connect(ip, port)

            if result != RobotErrorType.RobotError_SUCC:
                logger.info("connect server{0}:{1} failed.".format(ip, port))
            else:
                while True:
                    time.sleep(2)
                    waypoint = robot.get_current_waypoint()
                    print(waypoint)
                    print("----------------------------------------------" )


                    # Disconnect server link
                robot.disconnect()

        except RobotError as e:
            logger.error("robot Event:{0}".format(e))

        except KeyboardInterrupt:
            # Disconnect server link
            if robot.connected:
                # Disconnect the robot arm link
                robot.disconnect()
            # Release library resources
            Auboi5Robot.uninitialize()
            print("get waypoint run end-------------------------")

def runWaypoint(queue):
    while True:
        # while not queue.empty():
        print(queue.get(True))


def test_process_demo():
    # Initialize logger
    logger_init()

    # Start test
    logger.info("{0} test beginning...".format(Auboi5Robot.get_local_time()))

    # system initialization
    Auboi5Robot.initialize()

    # Create a robotic arm control class
    robot = Auboi5Robot()

    # Create context
    handle = robot.create_context()

    # Print context
    logger.info("robot.rshd={0}".format(handle))

    try:

        # time.sleep(0.2)
        # process_get_robot_current_status = GetRobotWaypointProcess()
        # process_get_robot_current_status.daemon = True
        # process_get_robot_current_status.start()
        # time.sleep(0.2)

        queue = Queue()

        p = Process(target=runWaypoint, args=(queue,))
        p.start()
        time.sleep(5)
        print("process started.")

        # Link server
        #ip ='localhost'
        ip = '192.168.1.200'
        port = 8899
        result = robot.connect(ip, port)

        if result != RobotErrorType.RobotError_SUCC:
            logger.info("connect server{0}:{1} failed.".format(ip, port))
        else:
            robot.enable_robot_event()
            robot.init_profile()
            joint_maxvelc = (2.596177, 2.596177, 2.596177, 3.110177, 3.110177, 3.110177)
            joint_maxacc = (17.308779/2.5, 17.308779/2.5, 17.308779/2.5, 17.308779/2.5, 17.308779/2.5, 17.308779/2.5)
            robot.set_joint_maxacc(joint_maxacc)
            robot.set_joint_maxvelc(joint_maxvelc)
            robot.set_arrival_ahead_blend(0.05)
            while True:
                time.sleep(1)

                joint_radian = (0.541678, 0.225068, -0.948709, 0.397018, -1.570800, 0.541673)
                robot.move_joint(joint_radian, True)
                

                joint_radian = (55.5/180.0*pi, -20.5/180.0*pi, -72.5/180.0*pi, 38.5/180.0*pi, -90.5/180.0*pi, 55.5/180.0*pi)
                robot.move_joint(joint_radian, True)

                joint_radian = (0, 0, 0, 0, 0, 0)
                robot.move_joint(joint_radian, True)

                print("-----------------------------")

                queue.put(joint_radian)

                # time.sleep(5)

                # process_get_robot_current_status.test()

                # print("-----------------------------")

                # Disconnect server link
            robot.disconnect()

    except KeyboardInterrupt:
        robot.move_stop()

    except RobotError as e:
        logger.error("robot Event:{0}".format(e))



    finally:
        # Disconnect server link
        if robot.connected:
            # Disconnect the robot arm link
            robot.disconnect()
        # Release library resources
        Auboi5Robot.uninitialize()
        print("run end-------------------------")

if __name__ =='__main__':
    #test_process_demo()
    
    # Initialize logger
    logger_init()

    # Start test
    logger.info("{0} test beginning...".format(Auboi5Robot.get_local_time()))

    # system initialization
    Auboi5Robot.initialize()

    # Create a robotic arm control class
    robot = Auboi5Robot()

    # Create context
    handle = robot.create_context()

    # Print context
    logger.info("robot.rshd={0}".format(handle))

    try:
        # Link server
        #ip ='localhost'
        ip = '192.168.1.200'
        port = 8899
        result = robot.connect(ip, port)

        if result != RobotErrorType.RobotError_SUCC:
            logger.info("connect server{0}:{1} failed.".format(ip, port))
        else:
            #while True:
                time.sleep(2)
                waypoint = robot.get_current_waypoint()
                print(waypoint)
                print("----------------------------------------------" )
                
                # parse waypoint into pos and rpy
                # get pos and rpy out from dictionary waypoint
                pos = waypoint['pos']
                ori = waypoint['ori']
                joint = waypoint['joint']
                rpy = robot.quaternion_to_rpy(ori)
                # print("pos={0}, rpy={1}".format(pos, rpy))

                # move pos 2cm in x Axis
                pos[0] = pos[0] + 0.02

                # # 正解测试
                # fk_ret = robot.forward_kin((-0.000003, -0.127267, -1.321122, 0.376934, -1.570796, -0.000008))
                # logger.info(fk_ret)

                # # 逆解
                # joint_radian = (0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000)
                ik_result = robot.inverse_kin(joint, pos, ori)
                logger.info(ik_result)
                robot.move_line(ik_result['joint'])
                

                # move target 2cm in its own x axis


                rpy = (0,0,0)
                robot.move_to_target_in_cartesian(pos, rpy)

                # Disconnect server link
                robot.disconnect()

    except RobotError as e:
        logger.error("robot Event:{0}".format(e))

    except KeyboardInterrupt:
        # Disconnect server link
        if robot.connected:
            # Disconnect the robot arm link
            robot.disconnect()
        # Release library resources
        Auboi5Robot.uninitialize()
        print("get waypoint run end-------------------------")

    logger.info("test completed")





















