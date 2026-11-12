/* Copyright (c) 2025 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/*
 * This OpMode illustrates how to program your robot to drive field relative.  This means
 * that the robot drives the direction you push the joystick regardless of the current orientation
 * of the robot.
 *
 * This OpMode assumes that you have four mecanum wheels each on its own motor named:
 *   front_left_motor, front_right_motor, back_left_motor, back_right_motor
 *
 *   and that the left motors are flipped such that when they turn clockwise the wheel moves backwards
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 *
 */
@TeleOp(name = "Teleop Red Team", group = "Robot")
//@Disabled
public class TeleopRedTeam extends OpMode {
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    // This declares the four motors needed
    DcMotor frontLeftDrive;
    DcMotor frontRightDrive;
    DcMotor backLeftDrive;
    DcMotor backRightDrive;
    DcMotor winchMotor;
    DcMotor intakeMotor1;
    DcMotor intakeMotor2;
    private AnalogInput laserAnalog1, laserAnalog2;
    private DistanceSensor distanceRight, distanceLeft;
    private DistanceSensing distanceSensing;

    private DigitalChannel magnetSensor;
    // Sensor scale: 3.3V corresponds to 1000 mm
    private static final double MAX_VOLTS = 3.3;
    private static final double MAX_DISTANCE_MM = 1000.0;
    private static final int DISTANCE_FILTER_SIZE = 2;
    private double[][] distanceSamples = new double[4][DISTANCE_FILTER_SIZE];

    private static final double IDEAL_SHOOT_DISTANCE_MM = 300;

    private static final double POWER_LEVEL = 0.9; // 1.0 = full speed

    private static final boolean USE_ENCODERS = true;

    private enum CatapultState { IDLE, WINDING, DOWN, LAUNCHING }
    private CatapultState catapultState;
    private int winchLaunchStart;
    private double angle_integrator = 0;
    private boolean spitOutMode = false;

    public String teamColor() {
        return "RED";
    }

    // This declares the IMU needed to get the current direction the robot is facing
    IMU imu;
    private void setIntakeMotors(double speed){
        intakeMotor1.setPower(speed);
        intakeMotor2.setPower(speed);
    }
    @Override
    public void init() {
        intakeMotor1 = hardwareMap.get(DcMotor.class, "IntakeMotor");
        intakeMotor2 = hardwareMap.get(DcMotor.class, "IntakeMotor2");
        winchMotor = hardwareMap.get(DcMotor.class, "WinchMotor");
        magnetSensor = hardwareMap.get(DigitalChannel.class, "MagSensor");
        frontLeftDrive = hardwareMap.get(DcMotor.class, "FrontLeft");
        frontRightDrive = hardwareMap.get(DcMotor.class, "FrontRight");
        backLeftDrive = hardwareMap.get(DcMotor.class, "BackLeft");
        backRightDrive = hardwareMap.get(DcMotor.class, "BackRight");        // Map the analog device from the hardware configuration

        laserAnalog1 = hardwareMap.get(AnalogInput.class, "LaserAnalog1");
        laserAnalog2 = hardwareMap.get(AnalogInput.class, "LaserAnalog2");

        distanceLeft = hardwareMap.get(DistanceSensor.class, "Distance1");
        distanceRight = hardwareMap.get(DistanceSensor.class, "Distance2");


        // We set the left motors in reverse which is needed for drive trains where the left
        // motors are opposite to the right ones.
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        winchMotor.setDirection(DcMotor.Direction.REVERSE);
        intakeMotor2.setDirection(DcMotor.Direction.REVERSE);


        if( USE_ENCODERS ) {
            frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        // TODO: Figure out how to properly initialize this taking into account the 30-degree mounting angle

        imu = hardwareMap.get(IMU.class, "imu");
        // This needs to be changed to match the orientation on your robot
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.DOWN;

        RevHubOrientationOnRobot orientationOnRobot = new
                RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        // Initialize the distance sensing algorithm:
        distanceSensing = new DistanceSensing(laserAnalog1, laserAnalog2,
                distanceLeft, distanceRight, imu, telemetry);

        // Initialize the April tag camera:
        initAprilTag();
    }

    @Override
    public void loop() {
        // Read angle sensors:
        distanceSensing.processSensors();

        // Get stick inputs:
        double forward = -(gamepad1.left_stick_y * Math.abs(gamepad1.left_stick_y)) * POWER_LEVEL;
        double right = gamepad1.left_stick_x * Math.abs(gamepad1.left_stick_x) * POWER_LEVEL;
        double rotate = gamepad1.right_stick_x * Math.abs(gamepad1.right_stick_x) * POWER_LEVEL;

        // If you press the left bumper, you get a drive NOT from the point of view of the robot
        // (much unlike driving an RC vehicle)
        // Scratch that.
        // Left bumper is now APRIL tag alignment
        if (gamepad1.left_bumper) {
            //driveFieldRelative(forward, right, rotate);

            // Pressing BOTH BUMPERS will actually align. Left only will just display on telemetry
            AlineToAprilTag( !gamepad1.right_bumper );
            angle_integrator = 0;
        } else if (gamepad1.right_bumper) {
            // AUTO_AIMBOT MODE
            angle_integrator += distanceSensing.WallAngle_Deg;
            double rotatePID = (distanceSensing.WallAngle_Deg * -Calibration.LASER_ANGLE_GAIN_P)
                    + angle_integrator * -Calibration.LASER_ANGLE_GAIN_I;

            double forwardPID = (Calibration.LASER_TARGET_DIST_MM - distanceSensing.WallDist_mm)
                    * Calibration.LASER_DIST_GAIN_P;
            forward = Math.max(Math.min(forward, 0.5), -0.5); // Limit forward to half power

            drive(forwardPID, right, rotatePID);
            telemetry.addData("Forward PID", "%.2f", forward);
            telemetry.addData("Rotate PID", "%.2f", rotate);
        } else {
            // Apply parabolic transform!
            drive(forward, right, rotate);
            angle_integrator = 0;
        }

        /* ************* Winch State Machine *********** */

        // Idle state:
        if( catapultState == CatapultState.IDLE ){
            // Turn motor off
           if (gamepad1.b ){
               winchMotor.setPower(-0.5);
           }
           else {
               winchMotor.setPower(0);
           }
            // Check conditions
            if( gamepad1.y ){
                catapultState = CatapultState.WINDING;
            }
        }
        // Winding state
        else if( catapultState == CatapultState.WINDING ){
            // Turn motor on
            winchMotor.setPower(1);
            // Check conditions
            if( !magnetSensor.getState() ){
                catapultState = CatapultState.DOWN;
                winchLaunchStart = winchMotor.getCurrentPosition();
            }
            if( gamepad1.b ){
                catapultState = CatapultState.IDLE;
            }
        }
        // Down state
        else if( catapultState == CatapultState.DOWN ){
            // Apply holding torque to motor
            double winchPower = (winchMotor.getCurrentPosition()-winchLaunchStart)
                    * -Calibration.WINCH_HOLDING_GAIN_P + Calibration.WINCH_HOLDING_FEEDFORWARD;
            winchMotor.setPower(winchPower);
            // Reset "spit out mode"
            spitOutMode = false;
            // Check conditions
            if( gamepad1.x ){
                catapultState = CatapultState.LAUNCHING;
            }
            if( gamepad1.b ){
                catapultState = CatapultState.IDLE;
            }
        }
        // Launching state
        else if( catapultState == CatapultState.LAUNCHING ){
            // Turn motor on
            winchMotor.setPower(1);
            // Check conditions
            if( (winchMotor.getCurrentPosition() - winchLaunchStart) > Calibration.WINCH_LAUNCH_AMOUNT ){
                catapultState = CatapultState.WINDING; // Go back to winding unless canceled
            }
            if( gamepad1.b ){
                catapultState = CatapultState.IDLE;
            }
        }
        else {
            catapultState = CatapultState.IDLE;
        }

        /* ************* Intake Motor Logic *********** */


        if( gamepad1.a ){
            setIntakeMotors(1.0);
        }
        else if( gamepad1.left_stick_button ){
            setIntakeMotors(-1);
        }
        else {
            setIntakeMotors(0);
        }

        telemetry.update();

    }
    private void initAprilTag() {

        // Create the AprilTag processor the easy way.
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();

        // Create the vision portal the easy way.
        //visionPortal = VisionPortal.easyCreateWithDefaults(
        //    hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);
        VisionPortal.Builder builder = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new android.util.Size(1280, 800)) // match supported resolution
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .addProcessor(aprilTag);
        visionPortal = builder.build();


    }   // end method initAprilTag()

    private void AlineToAprilTag(boolean display_only) {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());
        boolean found = false;

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection instanceof AprilTagClusterDetection) {
                AprilTagClusterDetection clusterDet = (AprilTagClusterDetection) detection;
                String name = clusterDet.metadata.name;
                telemetry.addLine(String.format("\n==== Tag Cluster (%s)", name));
                telemetry.addLine(String.format("Percent tags found: %d", clusterDet.percentClusterFound));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));

                if( name.contains(teamColor()) ){
                    double rotate = (Calibration.APRIL_TARGET_YAW - detection.ftcPose.yaw)
                            * Calibration.APRIL_YAW_GAIN_P;
                    double forward = (Calibration.APRIL_TARGET_DISTANCE - detection.ftcPose.range)
                            * -Calibration.APRIL_DIST_GAIN_P;
                    telemetry.addLine(String.format("rotate: %3.2f, forward: %3.2f", rotate, forward));
                    if (!display_only) {
                        drive(forward, 0, rotate);
                    } else {
                        drive(0, 0, 0);
                    }
                    found = true;
                }
            }
        }   // end for() loop

        // Set zero power if no april tags found:
        if( !found || display_only){
            drive(0, 0, 0);
        }

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");

        telemetry.update();
    }

    // This routine drives the robot field relative
    private void driveFieldRelative(double forward, double right, double rotate) {
        // First, convert direction being asked to drive to polar coordinates
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(right, forward);

        // Second, rotate angle by the angle the robot is pointing
        theta = AngleUnit.normalizeRadians(theta -
                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

        // Third, convert back to Cartesian
        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);

        // Finally, call the drive method with robot relative forward and right amounts
        drive(newForward, newRight, rotate);
    }

    // Thanks to FTC16072 for sharing this code!!
    public void drive(double forward, double right, double rotate) {

        double maxPower = 1;
        double maxSpeed = 1;  // make this slower for outreaches
//        if (gamepad1.right_bumper) {
//            maxPower = 0.6;
//            maxSpeed = 0.6;
//            forward = forward * 0.3;
//            right = right * 0.6;
//            //rotate = rotate * 0.4;
//        }

        // This calculates the power needed for each wheel based on the amount of forward,
        // strafe right, and rotate
        double frontLeftPower = forward + right + rotate;
        double frontRightPower = forward - right - rotate;
        double backRightPower = forward + right - rotate;
        double backLeftPower = forward - right + rotate;


        // This is needed to make sure we don't pass > 1.0 to any wheel
        // It allows us to keep all the motors in proportion to what they should
        // be and not get clipped
        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));

        // We multiply by maxSpeed so that it can be set lower for outreaches
        // When a young child is driving the robot, we may not want to allow full
        // speed.
        frontLeftDrive.setPower(maxSpeed * (frontLeftPower / maxPower));
        frontRightDrive.setPower(maxSpeed * (frontRightPower / maxPower));
        backLeftDrive.setPower(maxSpeed * (backLeftPower / maxPower));
        backRightDrive.setPower(maxSpeed * (backRightPower / maxPower));
    }
}
