package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class DistanceSensing {
    // Settings:
    private static final int FILTER_SIZE = 2;
    private double[][] distanceSamples = new double[4][FILTER_SIZE];
    // State variables
    public double WallAngle_Deg = 0, WallAngle_Rads = 0;
    public double WallDist_mm = 0;
    public double LeftDist_mm = 0;
    public double RightDist_mm = 0;
    private static final double MAX_VOLTS = 3.3 * 800/1000;
    private static final double MAX_DISTANCE_MM = 1000.0;
    private static final double VALID_WALL_DISTANCE_MM = 9900.0; // Disabled by setting high
    public boolean WallAngleValid = false;
    private double imuAngleWhenLastValid = 0, wallAngleWhenLastValid = 0;
    private boolean lastWallAngleValid = false;
    private final AnalogInput laserAnalog1, laserAnalog2;
    private final DistanceSensor distanceLeft, distanceRight;
    private final Telemetry telemetry;
    private final IMU imu;

    public DistanceSensing(AnalogInput analog1, AnalogInput analog2,
                           DistanceSensor distLeft, DistanceSensor distRight, IMU theImu,
                           Telemetry theTelemetry){
        laserAnalog1 = analog1;
        laserAnalog2 = analog2;
        distanceLeft = distLeft;
        distanceRight = distRight;
        imu = theImu;
        telemetry = theTelemetry;
    }

    public void processSensors()
    {

        double distance_mm1, distance_mm2, distanceRight_mm, distanceLeft_mm;

        // Read sensor voltage (0.0–3.3V)
        double volts1 = laserAnalog1.getVoltage();
        double volts2 = laserAnalog2.getVoltage();

        // Convert voltage to distance in millimeters (linear mapping)
        distance_mm1 = (volts1 / MAX_VOLTS) * MAX_DISTANCE_MM;
        distance_mm2 = (volts2 / MAX_VOLTS) * MAX_DISTANCE_MM;

        // Read the i2c digital sensors
        distanceLeft_mm = distanceLeft.getDistance(DistanceUnit.MM);
        distanceRight_mm = distanceRight.getDistance(DistanceUnit.MM);

        // Apply moving average filters.
        // First shift new readings into table:
        for( int i = 1; i < FILTER_SIZE; i ++ ){
            for( int j = 0; j < 4; j ++ ){
                distanceSamples[j][i] = distanceSamples[j][i-1];
            }
        }
        distanceSamples[0][0] = distance_mm1;
        distanceSamples[1][0] = distance_mm2;
        distanceSamples[2][0] = distanceLeft_mm;
        distanceSamples[3][0] = distanceRight_mm;

        // Then average top rows of table, and determine whether angle reading is valid:
        distance_mm1 = 0;
        distance_mm2 = 0;
        distanceLeft_mm = 0;
        distanceRight_mm = 0;
        WallAngleValid = true;
        for( int i = 0; i < FILTER_SIZE; i ++ ){
            distance_mm1 += distanceSamples[0][i];
            distance_mm2 += distanceSamples[1][i];
            distanceLeft_mm += distanceSamples[2][i];
            distanceRight_mm += distanceSamples[3][i];
            if( distanceSamples[0][i] > VALID_WALL_DISTANCE_MM || distanceSamples[1][i] > VALID_WALL_DISTANCE_MM ){
                WallAngleValid = false;
            }
        }
        distance_mm1 /= FILTER_SIZE;
        distance_mm2 /= FILTER_SIZE;
        distanceLeft_mm /= FILTER_SIZE;
        distanceRight_mm /= FILTER_SIZE;

        // Assign output variables:
        LeftDist_mm = distanceLeft_mm;
        RightDist_mm = distanceRight_mm;

        // Switch between IMU and angle sensors:
        double currentYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        if(WallAngleValid){
            // Calculate angle based on TRIGONOMETRY!!!
            WallAngle_Rads = Math.atan((distance_mm2-distance_mm1) / Calibration.LASER_DISTANCE_WIDTH_MM);
            WallAngle_Deg = WallAngle_Rads * 180 / 3.1415926535897932384626433;

            // Use trigonometry again to correct true wall distance from center of robot
            double hypotenuse = (distance_mm1 + distance_mm2)/2 + 9*25.4;
            WallDist_mm = hypotenuse * Math.abs(Math.cos(WallAngle_Rads));

            wallAngleWhenLastValid = WallAngle_Deg;
            imuAngleWhenLastValid = currentYaw;
        }
        else {
            // Angle sensors not valid. Use IMU dead reckoning instead (angle only)
            WallAngle_Deg = (currentYaw - imuAngleWhenLastValid) + wallAngleWhenLastValid;
        }

        telemetry.addData("Lasers", "%.1f / %.1f / %.1f / %.1f mm",
                distance_mm1, distance_mm2, distanceLeft_mm, distanceRight_mm);
        telemetry.addData("Wall Angle", "%.1f deg", WallAngle_Deg);

    }
}
