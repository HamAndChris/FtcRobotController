package org.firstinspires.ftc.teamcode;
// File: Calibration.java

/**
 * Central place for all calibration constants.
 * These values can be used across multiple OpModes and classes.
 */
public final class Calibration {

    // Prevent instantiation
    private Calibration() {}

    // Used to determine which APRIL tag to find:
    //public static final String TEAM_COLOR = "BLUE";
    //public static final String TEAM_COLOR = "RED";

    // Winch control calibrations:
    public static final int WINCH_LAUNCH_AMOUNT = 1500; // Encoder ticks to fire the winch
    public static final double WINCH_HOLDING_FEEDFORWARD = 0.15; // Holding motor power
    public static final double WINCH_HOLDING_GAIN_P = 0.001; // Motor holding proportional gain

    // APRIL tag aim-bot calibrations:
    public static final double APRIL_TARGET_DISTANCE = 36;
    public static final double APRIL_DIST_GAIN_P = 0.1;
    public static final double APRIL_TARGET_YAW = 0.0;
    public static final double APRIL_YAW_GAIN_P = 0.1;

    // Laser distance sensor calibrations:
    public static final double LASER_DISTANCE_WIDTH_MM = 9 * 25.4; // TODO: Measure more precisely
    public static final double LASER_ANGLE_GAIN_P = 1./800; // Laser angle alignment proportional gain
    public static final double LASER_ANGLE_GAIN_I = 0.0; // Laser angle alignment proportional gain
    public static final double LASER_TARGET_DIST_MM = 300;
    public static final double LASER_DIST_GAIN_P = 1./350;
}

