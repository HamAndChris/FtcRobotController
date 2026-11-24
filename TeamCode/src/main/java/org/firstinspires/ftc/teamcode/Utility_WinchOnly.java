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
import com.qualcomm.robotcore.eventloop.opmode.Utility;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

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
@Utility(name = "Winch motor only")
//@Disabled
public class Utility_WinchOnly extends OpMode {
    // This declares the four motors needed
    DcMotor winchMotor;

    @Override
    public void init() {
        winchMotor = hardwareMap.get(DcMotor.class, "WinchMotor");
        winchMotor.setDirection(DcMotor.Direction.REVERSE);
        telemetry.update();
    }

    @Override
    public void loop() {
        telemetry.addLine("Press A or Right Trigger: Drive winch");
        telemetry.addLine("Press B or Left Trigger:  Reverse winch");
        telemetry.addLine("Press X:                  Reset encoder");
        telemetry.addLine();

        // Reset encoder if X is pressed:
        if( gamepad1.xWasPressed() ){
            winchMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            winchMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        // Choose power level:
        double power;
        if( gamepad1.left_trigger > 0.1 ) {
            power = -gamepad1.left_trigger;
        }
        else if( gamepad1.right_trigger > 0.1 ) {
            power = gamepad1.right_trigger;
        }
        else if (gamepad1.a) {
            power = 1.0;
        }
        else if (gamepad1.b) {
            power = -1.0;
        }
        else {
            power = 0;
        }
        winchMotor.setPower(power);

        telemetry.addData("Power", "%.1f", power);
        telemetry.addData("Encoder", "%d", winchMotor.getCurrentPosition());
        telemetry.update();
    }

}
