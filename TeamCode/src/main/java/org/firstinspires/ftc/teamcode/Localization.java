package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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
    DcMotorEx frontLeft, frontRight, backLeft, backRight;
    IMU imu;

    // Used for outputs:
    double newForward, newRight, newRotate;
    private double x, y, heading, headingOffset, rawYaw;
    private double omegaRadPerSec, robotVxInPerSec, robotVyInPerSec;

    // Tuning multipliers. Mecanum wheels slip, especially when strafing.
    // Drive 100" and multiply by (100 / measured). Do the same for a 100" strafe.
    private static final double FORWARD_MULTIPLIER = 1.0;
    private static final double STRAFE_MULTIPLIER  = 1.0;


    // Internal state variables:
    private double lastFl, lastFr, lastBl, lastBr, lastHeading, lastTimeNs;

    public Localization(HardwareMap hardwareMap, Telemetry telemetry){
        this( hardwareMap, telemetry, new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.RADIANS, 0) );
    }

    public Localization(HardwareMap hardwareMap, Telemetry telemetry, Pose2D startingPose){
        frontLeft  = hardwareMap.get(DcMotorEx.class, "FrontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "FrontRight");
        backLeft   = hardwareMap.get(DcMotorEx.class, "BackLeft");
        backRight  = hardwareMap.get(DcMotorEx.class, "BackRight");
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

        setPose(startingPose);
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
        heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        int cFl = frontLeft.getCurrentPosition();
        int cFr = frontRight.getCurrentPosition();
        int cBl = backLeft.getCurrentPosition();
        int cBr = backRight.getCurrentPosition();

        // Wheel travel since last update, inches
        double dFl = (cFl - lastFl) / Calibration.COUNTS_PER_INCH;
        double dFr = (cFr - lastFr) / Calibration.COUNTS_PER_INCH;
        double dBl = (cBl - lastBl) / Calibration.COUNTS_PER_INCH;
        double dBr = (cBr - lastBr) / Calibration.COUNTS_PER_INCH;
        lastFl = cFl; lastFr = cFr; lastBl = cBl; lastBr = cBr;

        // Robot-frame movement (mecanum forward kinematics)
        double dForward = (dFl + dFr + dBl + dBr) / 4.0 * FORWARD_MULTIPLIER;
        double dStrafe  = (-dFl + dFr + dBl - dBr) / 4.0 * STRAFE_MULTIPLIER;  // +left

        // Heading comes from the IMU, not from the wheels
        // This is the ONLY IMU (I2C) read per loop.
        rawYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);  // CCW positive
        double newHeading = wrapAngle(rawYaw + headingOffset);
        double dTheta = wrapAngle(newHeading - lastHeading);

        // Integrate along a constant-curvature arc (more accurate than straight-line
        // when the robot is turning while it moves).
        double s, c;
        if (Math.abs(dTheta) < 1e-6) {
            s = 1.0 - dTheta * dTheta / 6.0;
            c = dTheta / 2.0;
        } else {
            s = Math.sin(dTheta) / dTheta;
            c = (1.0 - Math.cos(dTheta)) / dTheta;
        }
        double localX = dForward * s - dStrafe * c;
        double localY = dForward * c + dStrafe * s;

        // Rotate from robot frame into field frame using the heading at the START of the step
        double cosH = Math.cos(lastHeading), sinH = Math.sin(lastHeading);
        x += localX * cosH - localY * sinH;
        y += localX * sinH + localY * cosH;
        lastHeading = heading;

        // Velocities (robot frame) from encoder velocity + heading change
        long now = System.nanoTime();
        double dt = (now - lastTimeNs) / 1e9;
        lastTimeNs = now;
        double vFl = frontLeft.getVelocity() / Calibration.COUNTS_PER_INCH;
        double vFr = frontRight.getVelocity() / Calibration.COUNTS_PER_INCH;
        double vBl = backLeft.getVelocity() / Calibration.COUNTS_PER_INCH;
        double vBr = backRight.getVelocity() / Calibration.COUNTS_PER_INCH;
        robotVxInPerSec = (vFl + vFr + vBl + vBr) / 4.0 * FORWARD_MULTIPLIER;
        robotVyInPerSec = (-vFl + vFr + vBl - vBr) / 4.0 * STRAFE_MULTIPLIER;
        if (dt > 1e-4) omegaRadPerSec = dTheta / dt;
    }
    // ----------------------- Setting the pose -----------------------

    /** Set the robot's pose (start of auto, wall-alignment reset, AprilTag correction, etc.). */
    public void setPose(Pose2D p) {
        lastFl = frontLeft.getCurrentPosition();
        lastFr = frontRight.getCurrentPosition();
        lastBl = backLeft.getCurrentPosition();
        lastBr = backRight.getCurrentPosition();

        // Uses the yaw cached by the most recent update(); no IMU read here.
        headingOffset = wrapAngle(p.getHeading(AngleUnit.RADIANS) - rawYaw);

        x = p.getX(DistanceUnit.INCH);
        y = p.getY(DistanceUnit.INCH);
        heading = wrapAngle(p.getHeading(AngleUnit.RADIANS));
        lastHeading = heading;
        lastTimeNs = System.nanoTime();
        robotVxInPerSec = robotVyInPerSec = omegaRadPerSec = 0;
    }

    public void setPose(double xInches, double yInches, double headingDeg) {
        setPose(new Pose2D(DistanceUnit.INCH, xInches, yInches, AngleUnit.DEGREES, headingDeg));
    }

    /** Zero the heading to the current direction, keeping X/Y ("reset field-centric" button). */
    public void resetHeading() {
        setPose(new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.RADIANS, 0));
    }

    // ----------------------- Pose access -----------------------

    public Pose2D getPose() { return new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.RADIANS, heading); }

    public double getX()          { return x; }
    public double getY()          { return y; }
    public double getHeading()    { return heading; }                    // radians
    public double getHeadingDeg() { return Math.toDegrees(heading); }

    /** Robot-relative velocity: forward (in/s). */
    public double getRobotVx()  { return robotVxInPerSec; }
    /** Robot-relative velocity: left (in/s). */
    public double getRobotVy()  { return robotVyInPerSec; }
    /** Turn rate (rad/s), CCW positive. */
    public double getOmega()    { return omegaRadPerSec; }

    /** Field-relative X velocity (in/s). */
    public double getFieldVx() {
        return robotVxInPerSec * Math.cos(heading) - robotVyInPerSec * Math.sin(heading);
    }
    /** Field-relative Y velocity (in/s). */
    public double getFieldVy() {
        return robotVxInPerSec * Math.sin(heading) + robotVyInPerSec * Math.cos(heading);
    }

    /** Wrap an angle to (-PI, PI]. */
    private static double wrapAngle(double a) {
        while (a > Math.PI)  a -= 2.0 * Math.PI;
        while (a <= -Math.PI) a += 2.0 * Math.PI;
        return a;
    }
}
