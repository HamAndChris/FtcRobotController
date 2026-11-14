package org.firstinspires.ftc.teamcode;
// File: Calibration.java

/**
 * Central place for all calibration constants.
 * These values can be used across multiple OpModes and classes.
 */
public final class Calibration {

    // Prevent instantiation
    private Calibration() {}

    // Winch control calibrations:
    public static final int WINCH_LAUNCH_AMOUNT = 1500; // Encoder ticks to fire the winch
    public static final double WINCH_HOLDING_FEEDFORWARD = 0.15; // Holding motor power
    public static final double WINCH_HOLDING_GAIN_P = 0.001; // Motor holding proportional gain

    // User controls calibrations:
    public static final double STICK_RATE_LIMIT = 4.0; // Maximum command change per second (to smooth out controls)

    // APRIL tag aim-bot calibrations:
    public static final double APRIL_TARGET_DISTANCE = 54; // Adjust this to make the robot aim closer/farther from the hive
    public static final double APRIL_DIST_GAIN_P = 0.1;
    public static final double APRIL_TARGET_YAW = 4.0;  // Adjust this to make robot aim to the left/right
    public static final double APRIL_YAW_GAIN_P = 0.05; // was 0.1 when we were using yaw instead of bearing

    // Laser distance sensor calibrations:
    public static final double LASER_DISTANCE_WIDTH_MM = 15.375 * 25.4; // TODO: Measure more precisely
    public static final double LASER_ANGLE_GAIN_P   = 1./800; // Laser angle alignment proportional gain
    public static final double LASER_ANGLE_GAIN_I   = 0.0;    // Laser angle alignment integral gain
    public static final double LASER_TARGET_DIST_MM = 22 * 25.4; // Target distance from center of robot to wall during strafing
    public static final double LASER_DIST_GAIN_P    = 1./500;
    public static final double LASER_DIST_MAXPOWER  = 0.3;
    public static final double LASER_SAMPLE_RATE    = 5;         // How often to sample the side lasers in autonomous (Hz)

    // Autonomous behavior calibrations:
    public static final double AUTON_STRAFE_SPEED = 0.3;
    public static final double AUTON_ROTATE_SPEED = 0.3;
    public static final double AUTON_STRAFE_DIST  = 22 * 25.4; // Distance from center of robot to wall when strafing in autonomous
    public static final double AUTON_STEP_1_DIST  = 18 * 25.4; // How far to come off wall before beginning to look for APRIL tags
    public static final double AUTON_ANGLE_GAIN_P = 0.3/90; // IMU angle alignment proportional gain

    // LED control values from documentation:
    public static final double LED_OFF    = 0.00;
    public static final double LED_RED    = 0.27;
    public static final double LED_YELLOW = 0.38;
    public static final double LED_GREEN  = 0.50;
    public static final double LED_BLUE   = 0.61;
    public static final double LED_PURPLE = 0.72;
    public static final double LED_WHITE  = 0.85;

    // LED control setting based on behavior:
    public static final double LED_WINCH_UP      = LED_OFF;
    public static final double LED_WINCH_READY   = LED_GREEN;
    public static final double LED_AIMING        = LED_YELLOW;
    public static final double LED_READY_TO_FIRE = LED_RED;

    // Flower Bar positions:
    public static final double FLOWER_BAR_UP = 0.18;
    public static final double FLOWER_BAR_DOWN = 0.55;


    // Calculate the COUNTS_PER_INCH for your specific drive train.
    public static final double WHEEL_DIAMETER_INCHES  = 4.094; // For figuring circumference
    public static final double COUNTS_PER_OUTPUT_REV  = 537.7; // From datasheet
    public static final double COUNTS_PER_INCH        = (COUNTS_PER_OUTPUT_REV) /
                                                       (WHEEL_DIAMETER_INCHES * 3.1415);
}

