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
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.AprilSensing;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.Dictionary;
import java.util.Hashtable;
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
@TeleOp(name = "Teleop: RED", group = "Robot")
//@Disabled
public class TeleopRedTeam extends OpMode {
    // Hardware objects declared here:
    DcMotor frontLeftDrive;
    DcMotor frontRightDrive;
    DcMotor backLeftDrive;
    DcMotor backRightDrive;
    DcMotor winchMotor;
    DcMotor intakeMotor1;
    DcMotor intakeMotor2;
    AnalogInput laserAnalog1, laserAnalog2;
    DistanceSensor distanceRight, distanceLeft;
    DigitalChannel magnetSensor;
    Servo ledIndicator, flowerBar;
    IMU imu;

    // Helper files:
    private Distances         distances;
    private AprilTagProcessor aprilTag;
    private VisionPortal      visionPortal;
    private AprilSensing      aprilSensing;

    // Constants:
    private static final double POWER_LEVEL = 0.9; // 1.0 = full speed

    // Winch state machine variables:
    private enum CatapultState { IDLE, WINDING, DOWN, LAUNCHING }
    private CatapultState catapultState = CatapultState.WINDING;
    private int winchLaunchStart;

    // PID variables:
    private double angle_integrator = 0;

    // Rate limiting variables:
    private double forwardLimited=0, rightLimited=0, rotateLimited=0, lastTime=0;

    // Other variables:
    private volatile Gamepad operatorGamepad; // Gamepad used for function buttons
    double flowerBarPosition = 0.5;
    Dictionary<HardwareDevice, Double> cachedMotorPowers = new Hashtable<>();

    // Which APRIL tag to aim for. Overridden in subclass TeleopBlueTeam.java
    public String teamColor() {
        return "RED";
    }

    private void setIntakeMotors(double speed){
        intakeMotor1.setPower(speed);
        intakeMotor2.setPower(speed);
    }

    @Override
    public void init() {
        // Map hardware objects:
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
        ledIndicator = hardwareMap.get(Servo.class, "Indicator");
        flowerBar    = hardwareMap.get(Servo.class, "FlowerBar");

        imu = hardwareMap.get(IMU.class, "imu");

        // We set the left motors in reverse which is needed for drive trains where the left
        // motors are opposite to the right ones.
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        winchMotor.setDirection(DcMotor.Direction.REVERSE);
        intakeMotor2.setDirection(DcMotor.Direction.REVERSE);

        // Enable encoder velocity drive mode:
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Initialize the distance sensing algorithm:
        distances = new Distances(laserAnalog1, laserAnalog2,
                distanceLeft, distanceRight, imu, telemetry);

        // Initialize the April tag camera:
        aprilSensing = new AprilSensing(teamColor(), hardwareMap, telemetry);
    }

    @Override
    public void init_loop() {
        // Keep the winch wound when the user initialized the robot
        doWinchStatemachine(false);
    }

    @Override
    public void loop() {
        // Decide if we're using one gamepad or two:
        if( isGamepadConnected(gamepad2) ){
            telemetry.addLine("Dual Gamepad Mode");
            operatorGamepad = gamepad2;
        }
        else {
            telemetry.addLine("Single Gamepad Mode");
            operatorGamepad = gamepad1;
        }
        // Read angle sensors:
        distances.processSensors();

        // Get chassis stick inputs, applying parabolic transform:
        double forward = -(gamepad1.left_stick_y * Math.abs(gamepad1.left_stick_y)) * POWER_LEVEL;
        double right = gamepad1.left_stick_x * Math.abs(gamepad1.left_stick_x) * POWER_LEVEL;
        double rotate = gamepad1.right_stick_x * Math.abs(gamepad1.right_stick_x) * POWER_LEVEL;

        // Apply rate limits to smooth out robot motion:
        forwardLimited = applyRateLimit(forward, forwardLimited, Calibration.STICK_RATE_LIMIT);
        rightLimited   = applyRateLimit(right,   rightLimited,   Calibration.STICK_RATE_LIMIT);
        rotateLimited  = applyRateLimit(rotate,  rotateLimited,  Calibration.STICK_RATE_LIMIT);

        // Left bumper is now APRIL tag AUTO AIMBOT MODE
        if (gamepad1.left_bumper) {
            // Pressing BOTH BUMPERS will actually align. Left only will just display on telemetry
            AlineToAprilTag( !gamepad1.right_bumper );
            angle_integrator = 0;
        } 
        else if (gamepad1.right_bumper) {
            // Wall following mode
            angle_integrator += distances.WallAngle_Deg;
            double rotatePID = (distances.WallAngle_Deg * -Calibration.LASER_ANGLE_GAIN_P)
                    + angle_integrator * -Calibration.LASER_ANGLE_GAIN_I;

            double forwardPID = (Calibration.LASER_TARGET_DIST_MM - distances.WallDist_mm)
                    * Calibration.LASER_DIST_GAIN_P;
            // Limit forward PID to low power:
            forwardPID = Math.max(Math.min(forwardPID, Calibration.LASER_DIST_MAXPOWER), -Calibration.LASER_DIST_MAXPOWER);
            
            // Correct forward/right ratio with trigonometry to keep moving straight along the wall:
            double rightCorrected = rightLimited * Math.cos(distances.WallAngle_Rads);
            forwardPID += rightLimited * Math.sin(distances.WallAngle_Rads);

            drive(forwardPID, rightCorrected, rotatePID);
            telemetry.addData("Forward PID", "%.2f", forwardPID);
            telemetry.addData("Rotate PID", "%.2f", rotatePID);
        } 
        else {
            // Just apply game stick inputs
            drive(forwardLimited, rightLimited, rotateLimited);
            angle_integrator = 0;
        }

        /* ************* Winch State Machine *********** */

        doWinchStatemachine(true);

        /* ************* Intake Motor Logic *********** */

        // Left trigger spits out, even if right trigger is still held:
        if( operatorGamepad.left_trigger > 0.2 ){
            setIntakeMotors(-1);
        }
        else if( operatorGamepad.right_trigger > 0.2 ){
            setIntakeMotors(1.0);
        }
        else {
            setIntakeMotors(0);
        }

        /* ************* Flower Bar Controls *********** */

        if( operatorGamepad.dpad_up ){
            flowerBarPosition = Calibration.FLOWER_BAR_UP;
            setPositionCached(flowerBar, flowerBarPosition);
        }
        if( operatorGamepad.dpad_down ){
            flowerBarPosition = Calibration.FLOWER_BAR_DOWN;
            setPositionCached(flowerBar, flowerBarPosition);
        }
        //telemetry.addData("Flower Bar", "%.2f", flowerBarPosition);

        // Send all telemetry to driver station:
        telemetry.update();

        // Track previous loop time for rate calculations:
        lastTime = time;
    }

    // Apply a single rate limit:
    private double applyRateLimit(double desired, double last, double rateLimit){
        // Calculate max allowed increase based on rate limit and passage of time:
        double maxIncrease = (time - lastTime) * rateLimit;

        // Allow only that much increase, but instant decrease back to zero:
        if( last > 0 ){
            return Math.max(Math.min(desired, last + maxIncrease), 0);
        } else {
            return Math.min(Math.max(desired, last - maxIncrease), 0);
        }
    }

    private void doWinchStatemachine(boolean allowLaunch) {
        // Idle state:
        if( catapultState == CatapultState.IDLE ){
            // Turn motor off
           if (operatorGamepad.b ){
               setPowerCached(winchMotor, -0.5);
           }
           else {
               setPowerCached(winchMotor, 0);
           }
            // Check conditions
            if( operatorGamepad.y ){
                catapultState = CatapultState.WINDING;
            }
        }
        // Winding state
        else if( catapultState == CatapultState.WINDING ){
            // Turn motor on
            setPowerCached(winchMotor, 1);
            // Check conditions
            if( !magnetSensor.getState() ){
                catapultState = CatapultState.DOWN;
                winchLaunchStart = winchMotor.getCurrentPosition();
            }
            if( operatorGamepad.b ){
                catapultState = CatapultState.IDLE;
            }
        }
        // Down state
        else if( catapultState == CatapultState.DOWN ){
            // Apply holding torque to motor
            double winchPower = (winchMotor.getCurrentPosition()-winchLaunchStart)
                    * -Calibration.WINCH_HOLDING_GAIN_P + Calibration.WINCH_HOLDING_FEEDFORWARD;
            setPowerCached(winchMotor, winchPower);

            // Check conditions
            if( operatorGamepad.x && allowLaunch ){
                catapultState = CatapultState.LAUNCHING;
            }
            if( operatorGamepad.b ){
                catapultState = CatapultState.IDLE;
            }
        }
        // Launching state
        else if( catapultState == CatapultState.LAUNCHING ){
            // Turn motor on
            setPowerCached(winchMotor, 1);
            // Check conditions
            if( (winchMotor.getCurrentPosition() - winchLaunchStart) > Calibration.WINCH_LAUNCH_AMOUNT ){
                catapultState = CatapultState.WINDING; // Go back to winding unless canceled
            }
            if( operatorGamepad.b ){
                catapultState = CatapultState.IDLE;
            }
        }
        else {
            catapultState = CatapultState.IDLE;
        }
    }

    // Helper function to speed up loop time
    void setPowerCached(DcMotor motor, double power){
        // Get previous power value:
        Double prevPower = cachedMotorPowers.get(motor);
        if( prevPower == null ){
            prevPower = 0.0;
        }
        // Only update motor power if it changed by more than 2%, or stopping:
        if( Math.abs(power - prevPower) > 0.02 || (power == 0.0 && prevPower != 0.0) ) {
            motor.setPower(power);
            cachedMotorPowers.put(motor, power);
        }
    }

    // Helper function to speed up loop time
    void setPositionCached(Servo servo, double pos){
        // Get previous power value:
        Double prevPos = cachedMotorPowers.get(servo);
        if( prevPos == null ){
            prevPos = 0.0;
        }
        // Only update position if it changed by more than 2%:
        if( Math.abs(pos - prevPos) > 0.02 ) {
            servo.setPosition(pos);
            cachedMotorPowers.put(servo, pos);
        }
    }

    // Use the AprilSensing class to detect and choose alignment commands, then apply them along with the LED color:
    void AlineToAprilTag(boolean display_only) {
        aprilSensing.performSensing();
        if( !display_only ){
            drive(aprilSensing.Forward, aprilSensing.Right, aprilSensing.Rotate);
        }

        // Turn on LED:
        if( aprilSensing.OnTarget ){
            setPositionCached(ledIndicator, Calibration.LED_READY_TO_FIRE);
        }
        else if( aprilSensing.Detected ){
            setPositionCached(ledIndicator, Calibration.LED_AIMING);
        }
    }


    /**
     * Checks if a gamepad is connected based on its ID.
     */
    private boolean isGamepadConnected(Gamepad gp) {
        return gp != null && gp.id != Gamepad.ID_UNASSOCIATED;
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
        setPowerCached(frontLeftDrive, maxSpeed * (frontLeftPower / maxPower));
        setPowerCached(frontRightDrive, maxSpeed * (frontRightPower / maxPower));
        setPowerCached(backLeftDrive, maxSpeed * (backLeftPower / maxPower));
        setPowerCached(backRightDrive, maxSpeed * (backRightPower / maxPower));
    }
}
