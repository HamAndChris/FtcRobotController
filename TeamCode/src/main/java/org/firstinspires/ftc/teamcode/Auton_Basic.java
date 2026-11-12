/* Copyright (c) 2017 FIRST. All rights reserved.
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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.AprilSensing;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/* This OpMode will be the basis for all other Autonomous OpModes.
 * 
 * We will initialize all the inputs and outputs, but then just drive forward three inches.
 * Other Autonomous modes can extend this class and override the "doAuton" function.
*/

@Autonomous(name="Basic", group="Robot")
public class Auton_Basic extends LinearOpMode {

    // Hardware objects:
    DcMotor         frontLeftDrive   = null;
    DcMotor         frontRightDrive   = null;
    DcMotor         backLeftDrive   = null;
    DcMotor         backRightDrive   = null;
    DcMotor         intakeMotor1;
    DcMotor         intakeMotor2;
    DcMotor         winchMotor;
    AnalogInput     laserAnalog1, laserAnalog2;
    DistanceSensor  distanceRight, distanceLeft;
    DigitalChannel  magnetSensor;
    Servo           ledIndicator = null;
    Servo           flowerBar = null;
    IMU             imu;
    VisionPortal    visionPortal;

    // Helper files:
    AprilTagProcessor aprilTag;
    AprilSensing      aprilSensing;
    DistanceSensing   distanceSensing;

    // Winch state machine variables:
    enum CatapultState { IDLE, WINDING, DOWN, LAUNCHING }
    int           winchLaunchStart;
    CatapultState catapultState;

    // Other variables:
    final ElapsedTime runtime = new ElapsedTime();

    // Default team color:
    public String teamColor() {
        return "BLUE";
    }

    @Override
    public void runOpMode() {
        frontLeftDrive = hardwareMap.get(DcMotor.class, "FrontLeft");
        frontRightDrive = hardwareMap.get(DcMotor.class, "FrontRight");
        backLeftDrive = hardwareMap.get(DcMotor.class, "BackLeft");
        backRightDrive = hardwareMap.get(DcMotor.class, "BackRight");
        laserAnalog1 = hardwareMap.get(AnalogInput.class, "LaserAnalog1");
        laserAnalog2 = hardwareMap.get(AnalogInput.class, "LaserAnalog2");
        distanceLeft = hardwareMap.get(DistanceSensor.class, "Distance1");
        distanceRight = hardwareMap.get(DistanceSensor.class, "Distance2");
        magnetSensor = hardwareMap.get(DigitalChannel.class, "MagSensor");
        intakeMotor1 = hardwareMap.get(DcMotor.class, "IntakeMotor");
        intakeMotor2 = hardwareMap.get(DcMotor.class, "IntakeMotor2");

        // We set the left motors in reverse which is needed for drive trains where the left
        // motors are opposite to the right ones.
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Initialize the distance sensing algorithm:
        distanceSensing = new DistanceSensing(laserAnalog1, laserAnalog2,
                distanceLeft, distanceRight, imu, telemetry);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Starting at",  "%7d :%7d",
                frontLeftDrive.getCurrentPosition(),
                frontRightDrive.getCurrentPosition());
        telemetry.update();

        // Initialize the APRIL tag sensing / webcam
        aprilSensing = new AprilSensing(teamColor(), hardwareMap, telemetry);

        // Wait for the game to start (driver presses START)
        waitForStart();

        // Call the doAuton function, which will be overridden by other auton modes:
        doAuton();
    }

    // Function that performs the desired autonomous tasks. Override this to create new autons
    // without repeating all the setup code.
    public void doAuton()
    {
        // Step 1 move forward three inches
        double started = runtime.seconds();
        while( opModeIsActive() && distanceSensing.WallDist_mm < 3*25.4 && runtime.seconds() - started < 3 ) {
            distanceSensing.processSensors();
            drive(0.2, 0, 0);
        }

        // Step 2: Stop
        drive(0, 0, 0);

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);  // pause to display final telemetry message.
    }

    // Apply motor torques to follow the wall at the desired strafing speed
    void strafeAlongWall(double rightSpeed)
    {
        // Apply rotation control loop:
        double rotate = distanceSensing.WallAngle_Deg * -Calibration.LASER_ANGLE_GAIN_P;
        // Correct forward/right ratio to keep moving straight along wall:
        double right = Calibration.AUTON_STRAFE_SPEED * Math.cos(distanceSensing.WallAngle_Rads);
        double forward = Calibration.AUTON_STRAFE_SPEED * Math.sin(distanceSensing.WallAngle_Rads);
        // Apply distance control loop:
        forward += (Calibration.AUTON_STRAFE_DIST - distanceSensing.WallDist_mm) * Calibration.LASER_DIST_GAIN_P;
        drive(forward, right, rotate);
    }

    // Use the AprilSensing class to detect and choose alignment commands, then apply them along with the LED color:
    void AlineToAprilTag() {
        aprilSensing.performSensing();
        drive(aprilSensing.Forward, aprilSensing.Right, aprilSensing.Rotate); 
        if( aprilSensing.OnTarget ){
            ledIndicator.setPosition(Calibration.LED_READY_TO_FIRE);
        }
        else if( aprilSensing.Detected ){
            ledIndicator.setPosition(Calibration.LED_AIMING);
        }
    }
    
    // Drive both intake motors (1.0 full intake, -1.0 full reverse)
    void setIntakeMotors(double speed){
        intakeMotor1.setPower(speed);
        intakeMotor2.setPower(speed);
    }
    
    // Simplified version of the teleop winch state machine; no unwinding, or recovery from IDLE
    void doWinchStatemachine(boolean fireCatapult, boolean goToIdle) {
        // Idle state:
        if( catapultState == CatapultState.IDLE ){
            // Turn motor off
            winchMotor.setPower(0);

            // Check conditions
            // if( operatorGamepad.y ){
            //     catapultState = CatapultState.WINDING;
            // }
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
            if( goToIdle ){
                catapultState = CatapultState.IDLE;
            }
        }
        // Down state
        else if( catapultState == CatapultState.DOWN ){
            // Apply holding torque to motor
            double winchPower = (winchMotor.getCurrentPosition()-winchLaunchStart)
                    * -Calibration.WINCH_HOLDING_GAIN_P + Calibration.WINCH_HOLDING_FEEDFORWARD;
            winchMotor.setPower(winchPower);

            // Check conditions
            if( fireCatapult ){
                catapultState = CatapultState.LAUNCHING;
            }
            if( goToIdle ){
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
            if( goToIdle ){
                catapultState = CatapultState.IDLE;
            }
        }
        else {
            catapultState = CatapultState.IDLE;
        }

        telemetry.addData("Winch State", catapultState);
    }

    
    void drive(double forward, double right, double rotate)
    {
        double maxPower = 1;
        double maxSpeed = 1;  // make this slower for outreaches

        // This calculates the power needed for each wheel based on the amount of forward,
        // strafe right, and rotate
        double frontLeftPower = forward + right + rotate;
        double frontRightPower = forward - right - rotate;
        double backRightPower = forward + right - rotate;
        double backLeftPower = forward - right + rotate;


        // This is needed to make sure we don't pass > 1.0 to any wheel
        // It allows us to keep all of the motors in proportion to what they should
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


    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the OpMode running.
     */
    public void drivePosition(double speed,
                             //double leftInches, double rightInches,
                             double timeoutS, double forward, double right, double rotate) {
        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newBackLeftTarget;
        int newBackRightTarget;

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // This calculates the power needed for each wheel based on the amount of forward,
            // strafe right, and rotate
            double frontLeftInches = forward + right + rotate;
            double frontRightInches = forward - right - rotate;
            double backRightInches = forward + right - rotate;
            double backLeftInches = forward - right + rotate;

            // Determine new target position, and pass to motor controller
            //newFrontLeftTarget = frontLeftDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            //newBackLeftTarget = backLeftDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            //newFrontRightTarget = frontRightDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            //newBackRightTarget = backRightDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newFrontLeftTarget = frontLeftDrive.getCurrentPosition() + (int)(frontLeftInches * Calibration.COUNTS_PER_INCH);
            newBackLeftTarget = backLeftDrive.getCurrentPosition() + (int)(backLeftInches * Calibration.COUNTS_PER_INCH);
            newFrontRightTarget = frontRightDrive.getCurrentPosition() + (int)(frontRightInches * Calibration.COUNTS_PER_INCH);
            newBackRightTarget = backRightDrive.getCurrentPosition() + (int)(backRightInches * Calibration.COUNTS_PER_INCH);
            frontLeftDrive.setTargetPosition(newFrontLeftTarget);
            frontRightDrive.setTargetPosition(newFrontRightTarget);
            backLeftDrive.setTargetPosition(newBackLeftTarget);
            backRightDrive.setTargetPosition(newBackRightTarget);

            // Turn On RUN_TO_POSITION
            frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            frontLeftDrive.setPower(Math.abs(speed));
            frontRightDrive.setPower(Math.abs(speed));
            backLeftDrive.setPower(Math.abs(speed));
            backRightDrive.setPower(Math.abs(speed));


            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                   (runtime.seconds() < timeoutS) &&
                   (frontLeftDrive.isBusy() || frontRightDrive.isBusy() ||
                           backLeftDrive.isBusy() || backRightDrive.isBusy()       )) {

                // Display it for the driver.
                telemetry.addData("Running to",  " %7d :%7d", newFrontLeftTarget, newFrontRightTarget);
                telemetry.addData("Currently at",  " at %7d :%7d",
                                            frontLeftDrive.getCurrentPosition(), frontRightDrive.getCurrentPosition());
                telemetry.update();
            }
            // Allow settling time for encoder position
            sleep(250);

            // Stop all motion;
            frontLeftDrive.setPower(0);
            frontRightDrive.setPower(0);
            backLeftDrive.setPower(0);
            backRightDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //sleep(250);   // optional pause after each move.
        }
    }
}
