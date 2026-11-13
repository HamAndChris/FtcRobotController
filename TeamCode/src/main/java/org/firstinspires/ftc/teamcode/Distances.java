package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class Distances {
    // Settings:
    private static final int FILTER_SIZE = 2;
    private double[][] distanceSamples = new double[4][FILTER_SIZE];
    // State variables
    public double WallAngle_Deg = 0, WallAngle_Rads = 0;
    public double WallDist_mm = 0;
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

    // Private variables used for left/right distance sensing:
    private double leftDist_mm = 0, rightDist_mm = 0;
    private double lastLeftSample = 0, lastRightSample = 0; // Timestamp when last sampled
    private final ElapsedTime runtime = new ElapsedTime();

    // Constructor:
    public Distances(AnalogInput analog1, AnalogInput analog2,
                           DistanceSensor distLeft, DistanceSensor distRight, IMU theImu,
                           Telemetry theTelemetry){
        laserAnalog1 = analog1;
        laserAnalog2 = analog2;
        distanceLeft = distLeft;
        distanceRight = distRight;
        imu = theImu;
        telemetry = theTelemetry;

        // Initialize the IMU:
        // This needs to be changed to match the orientation on your robot

        // // Old "simple orintation" code:
        // RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
        //         RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD;
        // RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
        //         RevHubOrientationOnRobot.UsbFacingDirection.DOWN;

        // RevHubOrientationOnRobot orientationOnRobot = new
        //         RevHubOrientationOnRobot(logoDirection, usbDirection);
        // imu.initialize(new IMU.Parameters(orientationOnRobot));

        // New IMU orientation reflecting 30-degree back slope:
        Orientation hubRotation = new Orientation(
                AxesReference.INTRINSIC,
                AxesOrder.ZYX,
                AngleUnit.DEGREES,
                180,    // Z rotation
                0,      // Y rotation
                -120,    // X rotation (-90 - 30)
                0
        );

        RevHubOrientationOnRobot orientationOnRobot =
                new RevHubOrientationOnRobot(hubRotation);

        imu.initialize(new IMU.Parameters(orientationOnRobot));
    }

    // Sample the right distance sensor, only if it's been long enough since the last sample.
    // This improves performance because I2C reads slow everything down.
    public double getRight_mm(double sampleRate_Hz)
    {
        double samplePeriod = 1.0/sampleRate_Hz;
        double now = runtime.seconds();
        if( now - lastRightSample >= samplePeriod ){
            lastRightSample = now;
            rightDist_mm = distanceRight.getDistance(DistanceUnit.MM);
        }
        telemetry.addData("Right Laser", "%.1f mm", rightDist_mm);
        return rightDist_mm;
    }

    // Sample the left distance sensor, only if it's been long enough since the last sample.
    // This improves performance because I2C reads slow everything down.
    public double getLeft_mm(double sampleRate_Hz)
    {
        double samplePeriod = 1.0/sampleRate_Hz;
        double now = runtime.seconds();
        if( now - lastLeftSample >= samplePeriod ){
            lastLeftSample = now;
            leftDist_mm = distanceLeft.getDistance(DistanceUnit.MM);
        }
        telemetry.addData("Left Laser", "%.1f mm", leftDist_mm);
        return leftDist_mm;
    }

    // Shortcuts to use calibration value for sample rate:
    public double getRight_mm(){
        return getRight_mm(Calibration.LASER_SAMPLE_RATE);
    }
    public double getLeft_mm(){
        return getLeft_mm(Calibration.LASER_SAMPLE_RATE);
    }

    // Get the yaw angle from the IMU, inverted so that positive = RIGHT
    public double getIMUAngle(){
        return -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    // Process the rear sensors and calculate wall angle/distance.
    // (don't read side sensors unless requested since reading I2C is slow)
    public void processSensors()
    {
        double distance_mm1, distance_mm2, distanceRight_mm, distanceLeft_mm;

        // Read sensor voltage (0.0–3.3V)
        double volts1 = laserAnalog1.getVoltage();
        double volts2 = laserAnalog2.getVoltage();

        // Convert voltage to distance in millimeters (linear mapping)
        distance_mm1 = (volts1 / MAX_VOLTS) * MAX_DISTANCE_MM;
        distance_mm2 = (volts2 / MAX_VOLTS) * MAX_DISTANCE_MM;

        // Apply moving average filters.
        // First shift new readings into table:
        for( int i = 1; i < FILTER_SIZE; i ++ ){
            for( int j = 0; j < 2; j ++ ){
                distanceSamples[j][i] = distanceSamples[j][i-1];
            }
        }
        distanceSamples[0][0] = distance_mm1;
        distanceSamples[1][0] = distance_mm2;

        // Then average top rows of table, and determine whether angle reading is valid:
        distance_mm1 = 0;
        distance_mm2 = 0;
        WallAngleValid = true;
        for( int i = 0; i < FILTER_SIZE; i ++ ){
            distance_mm1 += distanceSamples[0][i];
            distance_mm2 += distanceSamples[1][i];
            if( distanceSamples[0][i] > VALID_WALL_DISTANCE_MM || distanceSamples[1][i] > VALID_WALL_DISTANCE_MM ){
                WallAngleValid = false;
            }
        }
        distance_mm1 /= FILTER_SIZE;
        distance_mm2 /= FILTER_SIZE;

        // Switch between IMU and angle sensors:
        if(WallAngleValid){
            // Calculate angle based on TRIGONOMETRY!!!
            WallAngle_Rads = Math.atan((distance_mm2-distance_mm1) / Calibration.LASER_DISTANCE_WIDTH_MM);
            WallAngle_Deg = WallAngle_Rads * 180 / 3.1415926535897932384626433;

            // Use trigonometry again to correct true wall distance from center of robot
            double hypotenuse = (distance_mm1 + distance_mm2)/2 + 9*25.4;
            WallDist_mm = hypotenuse * Math.abs(Math.cos(WallAngle_Rads));

            wallAngleWhenLastValid = WallAngle_Deg;
        }
        else {
            // Angle sensors not valid. Use IMU dead reckoning instead (angle only)
            double currentIMU = getIMUAngle();
            if( lastWallAngleValid ){
                imuAngleWhenLastValid = currentIMU;
            }
            WallAngle_Deg = (currentIMU - imuAngleWhenLastValid) + wallAngleWhenLastValid;
        }
        lastWallAngleValid = WallAngleValid;

        telemetry.addData("Rear Lasers", "%.1f mm, %.1f mm", distance_mm1, distance_mm2);
        telemetry.addData("Wall", "%.1f mm, %.1f deg", WallDist_mm, WallAngle_Deg);
    }
}
