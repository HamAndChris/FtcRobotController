package org.firstinspires.ftc.teamcode;
// File: Calibration.java

/**
 * Central place for all calibration constants.
 * These values can be used across multiple OpModes and classes.
 *
 * Note: If you change these and it fails to compile, it's because the ranges are being checked by a unit test
 *       which is located at TeamCode/src/test/java/org/firstinspires/ftc/teamcode/CalibrationTest.java
 */
public final class Calibration
{
    //=============================== Hardware Change Switches ===================================//
    public static final boolean DISABLE_I2C_DISTANCE_SENSORS = true;
    public static final boolean SINGLE_GAMEPAD_MODE          = false;
    public static final boolean USE_FIELD_RELATIVE           = false;

    //============================= Frequently Used Calibrations =================================//
    public static final double  TELEOP_POWER_LEVEL  = 1.0; // 1.0 = full speed
    public static final int     FLOWER_DRAIN_CYCLES = 3; // Number of times to move forward and back while draining flowers

    //============================ April Tag Aimbot Calibrations =================================//
    public static final double APRIL_TARGET_DISTANCE = 54; // Adjust this to make the robot aim closer/farther from the hive
    public static final double APRIL_DIST_GAIN_P = 0.1;
    public static final double APRIL_TARGET_BEARING = 0.0;    // Adjust this to make robot aim to the left/right (higher = more left)
    public static final double APRIL_YAW_GAIN_P = 0.05;       // was 0.1 when we were using yaw instead of bearing
    public static final double APRIL_CAMERA_INCLINE = 38;     // Degrees: How far below vertical camera is aimed (inclinometer on face of camera body)
    public static final double APRIL_CAMERA_HEIGHT = 12.125;  // Inches: Height of camera lens above ground
    public static final double APRIL_HIVE_MIN_HEIGHT = 50;    // Inches: How high the hive opening center must be to consider it "tipped up"

    //========================== Autonomous Behavior Calibrations ================================//
    /** Seconds: Stop all movement after this amount of time to avoid penalties */
    public static final double AUTON_TIME_LIMIT   = 30;
    /** Inches: How far to come off wall before beginning to look for APRIL tags */
    public static final double AUTON_DIST_STEP1   = 9;
    /** Inches: Distance to wall in parking zone at end of autonomy */
    public static final double AUTON_DIST_PARKING_REAR = 6;
    /** Inches: Distance to left wall in parking zone at end of autonomy, leftmost option */
    public static final double AUTON_DIST_PARKING_LEFT1 = 12;
    /** Inches: Distance to left wall in parking zone at end of autonomy, rightmost option */
    public static final double AUTON_DIST_PARKING_LEFT2 = 48;
    /** Inches: Distance from center of robot to wall when strafing */
    public static final double AUTON_STRAFE_DIST  = 16;
    /** Inches: Distance from center of robot to wall to approach flowers */
    public static final double AUTON_DIST_FLOWER_APPROACH = 20;
    /** Inches: Travel distance to drain flowers (subtract from FLOWER_APPROACH to get distance from wall) */
    public static final double AUTON_DIST_FLOWER_DRAIN = 8;

    // Autonomous motion profiling calibrations:

    /** MotorSpeed: Default wall-strafing speed for strafeAlongWall() operations */
    public static final double AUTON_STRAFE_SPEED   = 0.8;
    /** MotorSpeed: Translation speed for autoSlideTo() operations */
    public static final double AUTON_SLIDE_SPEED    = 0.8;
    /** MotorSpeed: Rotational speed for operations */
    public static final double AUTON_ROTATE_SPEED   = 0.5;
    /** MotorSpeed/sec: How quickly to ramp up/down translational movements */
    public static final double AUTON_ACCEL          = 0.8;
    /** MotorSpeed/sec: How quickly to ramp up/down rotational movements */
    public static final double AUTON_ROTATE_ACCEL   = 0.8;
    /** MotorSpeed/sec: How quickly to ramp up/down combined translational/rotational movements */
    public static final double AUTON_COMBINED_ACCEL  = 0.8;
    /** Inches: Distance for translational movements to be considered "at target" */
    public static final double AUTON_DIST_THRESHOLD = 2.0;
    /** InchesPerSec: Speed for translational movements to be considered "stopped" */
    public static final double AUTON_STOPPED_THRESHOLD = 1.0;
    /** DegreesPerSec: Speed for rotational movements to be considered "stopped" */
    public static final double AUTON_STOPPED_ROTATING_THRESHOLD = 10;
    /** Deg: IMU angle alignment "done" angle threshold */
    public static final double AUTON_ANGLE_THRESH   = 4.0;
    /** Sec: IMU angle alignment "done" time threshold */
    public static final double AUTON_ANGLE_SETTLING = 0.25;
    /** MotorSpeed/Deg: IMU angle alignment proportional gain */
    public static final double AUTON_ANGLE_GAIN_P = 0.5/45;
    /** Speed/inch: Position holding proportional gain */
    public static final double AUTON_POSITION_GAIN_P = 0.05;

    // Winch control calibrations:
    /** Encoder Counts: How far to rotate to fire the catapult */
    public static final int    WINCH_LAUNCH_AMOUNT = 1500;
    /** Motor Power: Starting amount to hold winch down */
    public static final double WINCH_HOLDING_FEEDFORWARD = 0.15;
    /** MotorPower/Counts: Motor holding proportional gain */
    public static final double WINCH_HOLDING_GAIN_P = 0.001;
    /** Seconds: How long after launch commanded to prevent any motion */
    public static final double WINCH_FIRING_LOCKOUT = 0.5;
    /** MotorPower: How much reverse power to apply when operator presses B */
    public static final double WINCH_UNWIND_POWER = 1.0;

    // Laser distance sensor calibrations:
    public static final double LASER_DISTANCE_WIDTH_MM = 12.375 * 25.4;
    public static final double LASER_ANGLE_GAIN_P   = 1./800; // Laser angle alignment proportional gain
    public static final double LASER_ANGLE_MAXPOWER = 0.5;
    public static final double LASER_TARGET_DIST_MM = 22 * 25.4; // Target distance from center of robot to wall during strafing
    public static final double LASER_DIST_GAIN_P    = 1./500;
    public static final double LASER_DIST_MAXPOWER  = 0.3;
    public static final double LASER_SAMPLE_RATE    = 5;         // How often to sample the side lasers in autonomous (Hz)

    // LED control values from documentation:
    public static final double LED_OFF    = 0.10;
    public static final double LED_RED    = 0.29;
    public static final double LED_YELLOW = 0.38;
    public static final double LED_GREEN  = 0.50;
    public static final double LED_BLUE   = 0.61;
    public static final double LED_PURPLE = 0.72;
    public static final double LED_WHITE  = 0.85;

    // LED control setting based on behavior:
//    public static final double LED_WINCH_UP      = LED_OFF;
//    public static final double LED_WINCH_READY   = LED_GREEN;
    public static final double LED_AIMING        = LED_YELLOW;
    public static final double LED_READY_TO_FIRE = LED_GREEN;

    // Flower Bar positions:
    public static final double FLOWER_BAR_UP = 0.36;
    public static final double FLOWER_BAR_DOWN = 0.81;


    // Calculate the COUNTS_PER_INCH for your specific drive train.
    public static final double WHEEL_DIAMETER_INCHES  = 4.094; // For figuring circumference
    public static final double COUNTS_PER_OUTPUT_REV  = 537.7; // From datasheet
    public static final double COUNTS_PER_INCH        = (COUNTS_PER_OUTPUT_REV)
                                                            / (WHEEL_DIAMETER_INCHES * 3.1415);
    public static final double MOTOR_MAX_SPEED_RPM    = 312; // From datasheet
    public static final double MAX_SPEED_INCHES_PER_S = (MOTOR_MAX_SPEED_RPM / 60)
                                                            * (WHEEL_DIAMETER_INCHES * 3.1415);

}

