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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

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
@TeleOp(name = "Robot: Field Relative Mecanum Drive", group = "Robot")
//@Disabled
public class RobotTeleopMecanumFieldRelativeDrive extends OpMode {
    // This declares the four motors needed
    DcMotor frontLeftDrive;
    DcMotor frontRightDrive;
    DcMotor backLeftDrive;
    DcMotor backRightDrive;
    DcMotor winchMotor;
    DcMotor intakeMotor;
    private AnalogInput laserAnalog1, laserAnalog2, laserAnalog3;
    private DistanceSensor distance1, distance2, distance3;
    private static final boolean USE_ANALOG_DISTANCE = false;

    private DigitalChannel magnetSensor;
    // Sensor scale: 3.3V corresponds to 1000 mm
    private static final double MAX_VOLTS = 3.3;
    private static final double MAX_DISTANCE_MM = 1000.0;
    private static final double LASER_DISTANCE_WIDTH_MM = 9 * 25.4; // TODO: Measure more precisely
    private static final int DISTANCE_FILTER_SIZE = 2;
    private double[][] distanceSamples = new double[3][DISTANCE_FILTER_SIZE];

    private static final double IDEAL_SHOOT_DISTANCE_MM = 300;

    private int catapultState;
    private int winchLaunchStart;
    private static final int WINCH_LAUNCH_AMOUNT = 1500;
    private double angle_integrator = 0;

    // This declares the IMU needed to get the current direction the robot is facing
    IMU imu;

    @Override
    public void init() {
        intakeMotor = hardwareMap.get(DcMotor.class, "IntakeMotor");
        winchMotor = hardwareMap.get(DcMotor.class, "WinchMotor");
        magnetSensor = hardwareMap.get(DigitalChannel.class, "MagSensor");
        frontLeftDrive = hardwareMap.get(DcMotor.class, "FrontLeft");
        frontRightDrive = hardwareMap.get(DcMotor.class, "FrontRight");
        backLeftDrive = hardwareMap.get(DcMotor.class, "BackLeft");
        backRightDrive = hardwareMap.get(DcMotor.class, "BackRight");        // Map the analog device from the hardware configuration
        if( USE_ANALOG_DISTANCE ) {
            laserAnalog1 = hardwareMap.get(AnalogInput.class, "LaserAnalog1");
            laserAnalog2 = hardwareMap.get(AnalogInput.class, "LaserAnalog2");
            laserAnalog3 = hardwareMap.get(AnalogInput.class, "LaserAnalog3");
        } else {
            distance1 = hardwareMap.get(DistanceSensor.class, "Distance1");
            distance2 = hardwareMap.get(DistanceSensor.class, "Distance2");
            distance3 = hardwareMap.get(DistanceSensor.class, "Distance3");
        }

        // We set the left motors in reverse which is needed for drive trains where the left
        // motors are opposite to the right ones.
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        winchMotor.setDirection(DcMotor.Direction.REVERSE);


        // This uses RUN_USING_ENCODER to be more accurate.   If you don't have the encoder
        // wires, you should remove these
        //frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu = hardwareMap.get(IMU.class, "imu");
        // This needs to be changed to match the orientation on your robot
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        RevHubOrientationOnRobot orientationOnRobot = new
                RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
    }

    @Override
    public void loop() {
        telemetry.addLine("Press A to reset Yaw");
        telemetry.addLine("Hold left bumper to drive in robot relative");
        telemetry.addLine("The left joystick sets the robot direction");
        telemetry.addLine("Moving the right joystick left and right turns the robot");

        // If you press the A button, then you reset the Yaw to be zero from the way
        // the robot is currently pointing
        if (gamepad1.a) {
            imu.resetYaw();
        }
        // quite fast...

        double distance_mm1, distance_mm2, distance_mm3;
        if( USE_ANALOG_DISTANCE ) {
            // Read sensor voltage (0.0–3.3V)
            double volts1 = laserAnalog1.getVoltage();
            double volts2 = laserAnalog2.getVoltage();
            double volts3 = laserAnalog3.getVoltage();

            // Convert voltage to distance in millimeters (linear mapping)
            distance_mm1 = (volts1 / MAX_VOLTS) * MAX_DISTANCE_MM;
            distance_mm2 = (volts2 / MAX_VOLTS) * MAX_DISTANCE_MM;
            distance_mm3 = (volts3 / MAX_VOLTS) * MAX_DISTANCE_MM;
        }
        else {
            distance_mm1 = distance1.getDistance(DistanceUnit.MM);
            distance_mm2 = distance2.getDistance(DistanceUnit.MM);
            distance_mm3 = distance3.getDistance(DistanceUnit.MM);
        }

        // Apply moving average filters.
        // First shift new readings into table:
        for( int i = 1; i < DISTANCE_FILTER_SIZE; i ++ ){
            for( int j = 0; j < 3; j ++ ){
                distanceSamples[j][i] = distanceSamples[j][i-1];
            }
        }
        distanceSamples[0][0] = distance_mm1;
        distanceSamples[1][0] = distance_mm2;
        distanceSamples[2][0] = distance_mm3;

        // Then average top rows of table:
        distance_mm1 = 0;
        distance_mm2 = 0;
        distance_mm3 = 0;
        for( int i = 0; i < DISTANCE_FILTER_SIZE; i ++ ){
            distance_mm1 += distanceSamples[0][i];
            distance_mm2 += distanceSamples[1][i];
            distance_mm3 += distanceSamples[2][i];
        }
        distance_mm1 /= DISTANCE_FILTER_SIZE;
        distance_mm2 /= DISTANCE_FILTER_SIZE;
        distance_mm3 /= DISTANCE_FILTER_SIZE;

        // Calculate angle based on TRIGONOMETRY!!!
        double wall_angle_rads = Math.atan((distance_mm2-distance_mm1) / LASER_DISTANCE_WIDTH_MM);
        double wall_angle_degs = wall_angle_rads * 180 / 3.1415926535897932384626433;
        double wall_dist = (distance_mm1 + distance_mm2)/2;

        // Telemetry
        //telemetry.addData("Voltage (V)", "%.3f", volts1);
        telemetry.addData("Distances (mm)", "%.1f, %.1f", distance_mm1, distance_mm2);
        telemetry.addData("Angle (deg)", "%.1f", wall_angle_degs);
        telemetry.addData("Mag Sensor", "%d", magnetSensor.getState() ? 1 : 0);


        // If you press the left bumper, you get a drive from the point of view of the robot
        // (much like driving an RC vehicle)
        if (gamepad1.left_bumper) {
            driveFieldRelative(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            angle_integrator = 0;
        } else if (gamepad1.right_bumper) {
            // AUTO_AIMBOT MODE
            angle_integrator += wall_angle_degs;
            double rotate = (wall_angle_degs * -1/100) + angle_integrator * -1/1000;

            double forward = (IDEAL_SHOOT_DISTANCE_MM - wall_dist) * 1/200;
            forward = Math.max(Math.min(forward, 0.5), -0.5);

            drive(forward, gamepad1.left_stick_x, rotate);
            telemetry.addData("Forward", "%.2f", forward);
            telemetry.addData("Rotate", "%.2f", rotate);
        } else {
            drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            angle_integrator = 0;
        }


        // Idle state:
        if( catapultState == 0 ){
            // Turn motor off
            winchMotor.setPower(0);
            // Check conditions
            if( gamepad1.y ){
                catapultState = 1;
            }
        }
        // Winding state
        if( catapultState == 1 ){
            // Turn motor on
            winchMotor.setPower(1);
            // Check conditions
            if( !magnetSensor.getState() ){
                catapultState = 2;
                winchLaunchStart = winchMotor.getCurrentPosition();
            }
            if( gamepad1.b ){
                catapultState = 0;
            }
        }
        // Down state
        if( catapultState == 2 ){
            // Turn motor off
            winchMotor.setPower(0);
            // Check conditions
            if( gamepad1.x ){
                catapultState = 3;
            }
        }
        // Launching state
        if( catapultState == 3 ){
            // Turn motor on
            winchMotor.setPower(1);
            // Check conditions
            if( (winchMotor.getCurrentPosition() - winchLaunchStart) > WINCH_LAUNCH_AMOUNT
                || gamepad1.b ){
                catapultState = 0;
            }
        }

        //Implement intake logic
        if(gamepad1.left_stick_button){
            intakeMotor.setDirection(DcMotor.Direction.REVERSE);
            intakeMotor.setPower(1);
        }
        else if( catapultState == 2 ){
            intakeMotor.setDirection(DcMotor.Direction.FORWARD);
            intakeMotor.setPower(1);
        }
       else {
            intakeMotor.setPower(0);
        }


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

        // Third, convert back to cartesian
        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);

        // Finally, call the drive method with robot relative forward and right amounts
        drive(newForward, newRight, rotate);
    }

    // Thanks to FTC16072 for sharing this code!!
    public void drive(double forward, double right, double rotate) {

        double maxPower = 1;
        double maxSpeed = 1;  // make this slower for outreaches
        if (gamepad1.right_bumper) {
            maxPower = 0.6;
            maxSpeed = 0.6;
            forward = forward * 0.3;
            right = right * 0.6;
            //rotate = rotate * 0.4;
        }

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
}
