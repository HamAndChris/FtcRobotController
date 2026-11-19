package org.firstinspires.ftc.teamcode;
// File: Calibration.java

/**
 * Central place for all calibration constants.
 * These values can be used across multiple OpModes and classes.
 */
public final class Calibration {
    // Prevent instantiation
    private Calibration() {}

    // Frequently used values:
    public static final boolean SINGLE_GAMEPAD_MODE = false;
    public static final boolean USE_FIELD_RELATIVE  = false;
    public static final double  TELEOP_POWER_LEVEL = 1.0; // 1.0 = full speed

    // Autonomous behavior calibrations:

    /** Seconds: Stop all movement after this amount of time to avoid penalties */
    public static final double AUTON_TIME_LIMIT = 30;
    /** MotorSpeed: Default wall-strafing speed for strafeAlongWall() operations */
    public static final double AUTON_STRAFE_SPEED = 0.3;
    /** MotorSpeed: Default translation speed for autoSlideTo() operations */
    public static final double AUTON_SLIDE_SPEED = 0.75;
    /** Deg/sec: Default rotational speed for operations */
    public static final double AUTON_ROTATE_SPEED = 90;
    /** MotorSpeed: Translated from angle speed */
    public static final double AUTON_ROTATE_MOTOR_SPEED = Localization.getRotateCommand(Math.toRadians(AUTON_ROTATE_SPEED));
    /** Inches: How far to come off wall before beginning to look for APRIL tags */
    public static final double AUTON_DIST_STEP1   = 9;
    /** Inches: Distance to wall in parking zone at end of autonomy */
    public static final double AUTON_DIST_PARKING = 6;
    /** Inches: Distance from center of robot to wall when strafing */
    public static final double AUTON_STRAFE_DIST  = 16;
    /** Inches: Distance from center of robot to wall to approach flowers */
    public static final double AUTON_DIST_FLOWER_APPROACH = 18;
    /** Inches: Travel distance to drain flowers */
    public static final double AUTON_DIST_FLOWER_DRAIN = 6;

    // Autonomous motion profiling calibrations:

    /** MotorSpeed/sec: How quickly to ramp up/down translational movements */
    public static final double AUTON_ACCEL          = 1.0;
    /** Inches: Distance for translational movements to be considered "at target" */
    public static final double AUTON_DIST_THRESHOLD = 1.0;
    /** Deg: IMU angle alignment "done" angle threshold */
    public static final double AUTON_ANGLE_THRESH   = 2.0;
    /** Deg/sec/sec: How quickly to ramp up/down rotational movements */
    public static final double AUTON_ANGLE_ACCEL    = 180;
    /** Sec: IMU angle alignment "done" time threshold */
    public static final double AUTON_ANGLE_SETTLING = 0.25;
    /** MotorSpeed/Deg: IMU angle alignment proportional gain */
    public static final double AUTON_ANGLE_GAIN_P = 0.3/90;
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

    // APRIL tag aim-bot calibrations:
    public static final double APRIL_TARGET_DISTANCE = 54; // Adjust this to make the robot aim closer/farther from the hive
    public static final double APRIL_DIST_GAIN_P = 0.1;
    public static final double APRIL_TARGET_BEARING = 4.0;  // Adjust this to make robot aim to the left/right
    public static final double APRIL_YAW_GAIN_P = 0.05; // was 0.1 when we were using yaw instead of bearing

    // Laser distance sensor calibrations:
    public static final double LASER_DISTANCE_WIDTH_MM = 15.375 * 25.4;
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
    public static final double FLOWER_BAR_UP = 0.18;
    public static final double FLOWER_BAR_DOWN = 0.65;


    // Calculate the COUNTS_PER_INCH for your specific drive train.
    public static final double WHEEL_DIAMETER_INCHES  = 4.094; // For figuring circumference
    public static final double COUNTS_PER_OUTPUT_REV  = 537.7; // From datasheet
    public static final double COUNTS_PER_INCH        = (COUNTS_PER_OUTPUT_REV)
                                                            / (WHEEL_DIAMETER_INCHES * 3.1415);
    public static final double MOTOR_MAX_SPEED_RPM    = 312; // From datasheet
    public static final double MAX_SPEED_INCHES_PER_S = (MOTOR_MAX_SPEED_RPM / 60)
                                                            * (WHEEL_DIAMETER_INCHES * 3.1415);

}

