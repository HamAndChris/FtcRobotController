package org.firstinspires.ftc.teamcode;

import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.assertTrue;

public class CalibrationTest {
    /** Ensure that all calibration values make physical sense */
    @Test public void testCalibrations(){
        // Check for values that are the incorrect sign:
        assertTrue(Calibration.AUTON_ACCEL > 0.01);
        assertTrue(Calibration.AUTON_ANGLE_ACCEL > 10);
        assertTrue(Calibration.AUTON_ANGLE_ACCEL > 10);
        assertTrue(Calibration.AUTON_ROTATE_MOTOR_SPEED > 0.05);
        assertTrue(Calibration.AUTON_SLIDE_SPEED > 0.05);
        assertTrue(Calibration.AUTON_STRAFE_SPEED > 0.05);
        assertTrue(Calibration.AUTON_ANGLE_SETTLING >= 0);
        assertTrue(Calibration.TELEOP_POWER_LEVEL >= 0.1);

        // Gains must be positive or zero:
        assertTrue(Calibration.AUTON_POSITION_GAIN_P >= 0);
        assertTrue(Calibration.AUTON_ANGLE_GAIN_P >= 0);
        assertTrue(Calibration.APRIL_DIST_GAIN_P >= 0);
        assertTrue(Calibration.APRIL_YAW_GAIN_P >= 0);
        assertTrue(Calibration.LASER_ANGLE_GAIN_P >= 0);
        assertTrue(Calibration.LASER_DIST_GAIN_P >= 0);
        assertTrue(Calibration.WINCH_HOLDING_GAIN_P >= 0);

        // Check for values that wouldn't make sense at all:
        assertTrue(Math.abs(Calibration.APRIL_TARGET_BEARING) < 20);
        assertTrue(Math.abs(Calibration.AUTON_ROTATE_MOTOR_SPEED) <= 1.0); // This one actually tests a function output
        assertTrue(Math.abs(Calibration.AUTON_SLIDE_SPEED) <= 1.0);
        assertTrue(Math.abs(Calibration.TELEOP_POWER_LEVEL) <= 1.0);
        assertTrue(Calibration.AUTON_ANGLE_THRESH > 0.1);
        assertTrue(Calibration.WINCH_FIRING_LOCKOUT < 2.0);
        assertTrue(Calibration.LASER_DISTANCE_WIDTH_MM < 18 * 25.4);
        assertTrue(Calibration.LASER_ANGLE_MAXPOWER > 0.05);

        // Check for values outside the limits of the sensor:
        double maxLaserDist = 1000/25.4;
        assertTrue(Calibration.AUTON_DIST_PARKING_REAR <= maxLaserDist);
        assertTrue(Calibration.AUTON_STRAFE_DIST <= maxLaserDist);
    }
}
