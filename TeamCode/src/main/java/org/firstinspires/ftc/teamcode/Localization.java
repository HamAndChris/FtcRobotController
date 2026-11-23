package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
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
    Telemetry telemetry;
    DcMotorEx frontLeft, frontRight, backLeft, backRight;
    IMU imu;

    // Used for outputs:
    double newForward, newRight, newRotate;
    private double x, y, heading, headingOffset, rawYaw;
    private double omegaRadPerSec, robotVxInPerSec, robotVyInPerSec;

    // Tuning multipliers. Mecanum wheels slip, especially when strafing.
    // Drive 100" and multiply by (100 / measured). Do the same for a 100" strafe.
    public static final double FORWARD_MULTIPLIER = 1.005;
    public static final double STRAFE_MULTIPLIER  = 0.92;
    public static final double ROTATE_MULTIPLIER  = 0.38;

    // Some geometry:
    public static final double TRACKWIDTH_INCH = 16; // May not be precise, but ROTATE_MULTIPLIER was calibrated at 16


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
        rawYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);  // CCW positive

        setPose(startingPose);
    }

    /** Enable testing of velocity calculations. NOT INTENDED FOR USE ON ROBOT */
    public void overrideRobotVelocity(double velX, double velY){
        robotVxInPerSec = velX;
        robotVyInPerSec = velY;
    }

    /** Return true if we need to start decelerating from our current speed to land on the target */
    public boolean inDecelRadius(double targetX, double targetY, double decelInchPerSec2){
        double distance = distanceTo(targetX, targetY);
        double currentSpeed = getRobotSpeed();

        // Calculate max speed command for smooth _deceleration_ from this distance using kinematics equation:
        double decelSpeed_inchPerSec = Math.sqrt( 2 * Math.abs(decelInchPerSec2) * distance );
        return currentSpeed >= decelSpeed_inchPerSec;
    }

    /** Take a desired field-relative X/Y command and convert to actual robot drive commands. */
    public void convertFieldRelative(double fwd_y, double right_x, double rotate)
    {
        // First, convert direction being asked to drive to polar coordinates
        double theta = Math.atan2(fwd_y, right_x);
        double r = Math.hypot(right_x, fwd_y);

        convertSpeedDirection(r, theta);
        newRotate = rotate;
    }

    // Take a desired speed and translation direction and convert to robot forward/right commands (ignores rotation)
    public void convertSpeedDirection(double speed, double directionRads)
    {
        // Rotate angle by the angle the robot is pointing
        double theta = AngleUnit.normalizeRadians(directionRads - heading);

        // Third, convert back to Cartesian
        newForward = speed * Math.cos(theta);
        newRight = -speed * Math.sin(theta); // Right is flipped because Y is meant to be left from X
    }


    // Get the yaw angle from the IMU, inverted so that positive = RIGHT
    public double getClockwiseAngle(){
        return - Math.toDegrees(heading);
    }

    // Return the current yaw angle in radians
    public double getYawRadians(){
        return heading;
    }

    // This may require more steps in the future:
    public void resetYaw(){
        //imu.resetYaw();
        resetHeading();
    }

    // Call this once per loop:
    public void update()
    {
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
        heading = wrapAngle(rawYaw + headingOffset);
        double dTheta = wrapAngle(heading - lastHeading);

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

        telemetry.addData("Position (x/y/angle)", "%.1f, %.1f inch, %.1f deg", x, y, getHeadingDeg());
    }
    // ----------------------- Setting the pose -----------------------

    public void setPose(double xInches, double yInches, double headingDeg) {
        setPose(new Pose2D(DistanceUnit.INCH, xInches, yInches, AngleUnit.DEGREES, headingDeg));
    }

    /** Set the robot's pose (start of auto, wall-alignment reset, AprilTag correction, etc.). */
    public void setPose(Pose2D p) {
        lastFl = frontLeft.getCurrentPosition();
        lastFr = frontRight.getCurrentPosition();
        lastBl = backLeft.getCurrentPosition();
        lastBr = backRight.getCurrentPosition();

        // Uses the yaw cached by the most recent update(); no IMU read here.
        //headingOffset = wrapAngle(p.getHeading(AngleUnit.RADIANS) - rawYaw);

        x = p.getX(DistanceUnit.INCH);
        y = p.getY(DistanceUnit.INCH);
        setHeadingDeg( p.getHeading(AngleUnit.DEGREES) );
        lastTimeNs = System.nanoTime();
        robotVxInPerSec = robotVyInPerSec = omegaRadPerSec = 0;
    }

    /** Set the robot heading in radians, preserving X/Y position. */
    public void setHeadingDeg(double headingDeg) {
        heading = wrapAngle(Math.toRadians(headingDeg));
        headingOffset = wrapAngle(heading - rawYaw);
        lastHeading = heading;
    }

    /** Zero the heading to the current direction, keeping X/Y ("reset field-centric" button). */
    public void resetHeading() {
        setPose(new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.RADIANS, 0));
    }

    // ----------------------- Pose access -----------------------

    public Pose2D getPose() { return new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.RADIANS, heading); }

    /** Get current X field position in inches */
    public double getX()          { return x; }
    /** Get current Y field position in inches */
    public double getY()          { return y; }
    /** Get current heading in CCW radians */
    public double getHeading()    { return heading; }
    /** Get current heading in CCW degrees */
    public double getHeadingDeg() { return Math.toDegrees(heading); }

    /** Robot-relative velocity: forward (in/s). */
    public double getRobotVx()  { return robotVxInPerSec; }
    /** Robot-relative velocity: left (in/s). */
    public double getRobotVy()  { return robotVyInPerSec; }
    /** Turn rate (rad/s), CCW positive. */
    public double getOmega()    { return omegaRadPerSec; }
    /** Turn rate (deg/s), CCW positive. */
    public double getOmegaDeg()    { return Math.toDegrees(omegaRadPerSec); }
    /** Robot overall velocity (inch/s). */
    public double getRobotSpeed()  {
        return Math.hypot(robotVyInPerSec, robotVxInPerSec);
    }

    /** Get heading in degrees, snapped to nearest cardinal direction (multiple of 90) */
    public double getHeadingSnappedTo(double increment){
        return Math.round( Math.abs(getHeadingDeg()) / increment ) * increment * Math.signum(heading);
    }

    /** Field-relative X velocity (in/s). */
    public double getFieldVx() {
        return robotVxInPerSec * Math.cos(heading) - robotVyInPerSec * Math.sin(heading);
    }
    /** Field-relative Y velocity (in/s). */
    public double getFieldVy() {
        return robotVxInPerSec * Math.sin(heading) + robotVyInPerSec * Math.cos(heading);
    }

    /** Calculate the difference in X, Y, and Heading from current location to another one. */
    public Pose2D differenceFrom( Pose2D targetPose ) {
        return new Pose2D(DistanceUnit.INCH,
                x - targetPose.getX(DistanceUnit.INCH),
                y - targetPose.getY(DistanceUnit.INCH),
                AngleUnit.RADIANS,
                wrapAngle(heading - targetPose.getHeading(AngleUnit.RADIANS)) );
    }

    /** Calculate difference in heading from desired. Returns CCW radians. */
    public double diffFromHeading(double desiredHeadingRads){
        return wrapAngle(heading - desiredHeadingRads);
    }
    /** Calculate difference in heading from desired. Returns CCW degrees. */
    public double diffFromHeadingDeg(double desiredHeadingDeg){
        return Math.toDegrees( wrapAngle(Math.toRadians(desiredHeadingDeg) - heading) );
    }

    /** Calculate X/Y distance from current location to another point. Ignores heading. */
    public double distanceTo( Pose2D targetPose ){
        return distanceTo( targetPose.getX(DistanceUnit.INCH), targetPose.getY(DistanceUnit.INCH) );
    }

    /** Calculate X/Y distance from current location to another point. Ignores heading. */
    public double distanceTo( double xInches, double yInches ){
        double xDiff = xInches - x;
        double yDiff = yInches - y;
        return Math.hypot(xDiff, yDiff);
    }

    /** Calculate heading from current location to another point (ignoring robot heading). Returns CCW radians. */
    public double directionTo( Pose2D targetPose ){
        return directionTo( targetPose.getX(DistanceUnit.INCH), targetPose.getY(DistanceUnit.INCH) );
    }

    /** Calculate heading from current location to another point (ignoring robot heading). Returns CCW radians. */
    public double directionTo( double xInches, double yInches ){
        double xDiff = xInches - x;
        double yDiff = yInches - y;
        return Math.atan2(yDiff, xDiff);
    }

    /** Get the rotation command for a desired angular rate in CCW radians/s */
    public static double findRotateCommand(double radiansPerSec){
        double desiredWheelSpeed_inchPerSec = Math.sin(radiansPerSec)*TRACKWIDTH_INCH/2;
        double desiredMotorSpeed = -desiredWheelSpeed_inchPerSec / Calibration.MAX_SPEED_INCHES_PER_S / ROTATE_MULTIPLIER;
        return Math.max(Math.min(desiredMotorSpeed, 1.0), -1.0);
    }

    /** Estimate the rotational speed that would result from a given "rotate" command.
     *  Note: rotational rate will be the same sign as rotateCmd, which means direction is inverted. */
    public static double calculateRotationRadPerSec(double rotateCmd){
        double decimate = 0.01; // Math.asin can only work with values less than 1
        double wheelSpeed_inchPerSec = rotateCmd * Calibration.MAX_SPEED_INCHES_PER_S * decimate;
        return Math.asin(wheelSpeed_inchPerSec / (TRACKWIDTH_INCH/2)) / decimate * ROTATE_MULTIPLIER;
    }

    /** Estimate the rotational speed that would result from a given "rotate" command.
     *  Note: rotational rate will be the same sign as rotateCmd, which means direction is inverted. */
    public static double calculateRotationDegPerSec(double rotateCmd){
        return Math.toDegrees(calculateRotationRadPerSec(rotateCmd));
    }

    /** Get the forward command for a desired forward speed in inches per second */
    public static double findForwardCommand(double inchesPerSec){
        return inchesPerSec / Calibration.MAX_SPEED_INCHES_PER_S / FORWARD_MULTIPLIER;
    }
    /** Get the forward command for a desired forward speed in inches per second */
    public static double findRightCommand(double inchesPerSec){
        return inchesPerSec / Calibration.MAX_SPEED_INCHES_PER_S / STRAFE_MULTIPLIER;
    }


    // ----------------------- Utilities -----------------------

    /** Wrap an angle to (-PI, PI]. */
    private static double wrapAngle(double a) {
        while (a > Math.PI)  a -= 2.0 * Math.PI;
        while (a <= -Math.PI) a += 2.0 * Math.PI;
        return a;
    }
}
