package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class Localization {
    public Pose2D pose = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);
    Telemetry telemetry;
    DcMotor frontLeft, frontRight, backLeft, backRight;
    IMU imu;

    // Used for outputs:
    double newForward, newRight, newRotate;

    public Localization(HardwareMap hardwareMap, Telemetry telemetry){
        frontLeft  = hardwareMap.get(DcMotor.class, "FrontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "FrontRight");
        backLeft   = hardwareMap.get(DcMotor.class, "BackLeft");
        backRight  = hardwareMap.get(DcMotor.class, "BackRight");
        imu        = hardwareMap.get(IMU.class, "imu");
        this.telemetry = telemetry;

        // Initialize the IMU:

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
        imu.resetYaw();
    }

    public void translateToFieldRelative(double forward, double right, double rotate)
    {
        // First, convert direction being asked to drive to polar coordinates
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(right, forward);

        // Second, rotate angle by the angle the robot is pointing
        theta = AngleUnit.normalizeRadians(theta - pose.getHeading(AngleUnit.RADIANS));

        // Third, convert back to Cartesian
        newForward = r * Math.sin(theta);
        newRight = r * Math.cos(theta);
        newRotate = rotate;
    }


    // Get the yaw angle from the IMU, inverted so that positive = RIGHT
    public double getClockwiseAngle(){
        return -pose.getHeading(AngleUnit.DEGREES);
    }

    // Return the current yaw angle in radians
    public double getYawRadians(){
        return pose.getHeading(AngleUnit.RADIANS);
    }

    // This may require more steps in the future:
    public void resetYaw(){
        imu.resetYaw();
    }

    // Call this once per loop:
    public void update(){
        // TODO: Implement odometry. For now just keep track of the IMU angle
        double imuYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        pose = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, imuYaw);
    }
}
