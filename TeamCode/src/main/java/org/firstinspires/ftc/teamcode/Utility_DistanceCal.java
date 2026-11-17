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
import com.qualcomm.robotcore.eventloop.opmode.Utility;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


@Utility(name="Calibrate Distance")
public class Utility_DistanceCal extends Auton_Basic {

    public static final double DISTANCE  = 100; // inches
    public static final double SPEED     = 0.2; // motor speed normalized (0 to 1.0)
    public static final double THRESHOLD = 0.1; // inches: considered "at target"
    public static final double ACCEL     = 0.1; // motor speed increase per second
    public static final double ANGLE_P   = 0.3/90; // Power/deg: Proportional gain to prevent rotation

    // Function that performs the desired autonomous tasks. Override this to create new autons
    // without repeating all the setup code.
    @Override
    public void doAuton()
    {
        double targetFwd = 0, targetRight = 0;

        // Wait for user to press the D-pad direction they wish to go:
        while( opModeIsActive() ){
            beginLoop("Waiting for input");
            telemetry.addLine("Press D-Pad to begin moving.");
            telemetry.addData("Settings", "%.1f inches, speed %0.1f, accel %0.2f", DISTANCE, SPEED, ACCEL);
            if( gamepad1.dpad_up ){
                targetFwd = DISTANCE;
                break;
            }
            if( gamepad1.dpad_down ){
                targetFwd = -DISTANCE;
                break;
            }
            if( gamepad1.dpad_right ){
                targetRight = DISTANCE;
                break;
            }
            if( gamepad1.dpad_left ){
                targetRight = -DISTANCE;
                break;
            }
        }

        // Reset the pose and time
        localization.setPose(0, 0, 0);
        runtime.reset();

        // Smoothly move the desired amount
        double currentForward = 0, currentRight = 0;
        double remaining = DISTANCE;
        double lastTime = runtime.seconds();
        while( opModeIsActive()
                && Math.abs(localization.getX()) < (DISTANCE - THRESHOLD)
                && Math.abs(localization.getY()) < (DISTANCE - THRESHOLD) ){
            beginLoop("Driving to target");
            double now = runtime.seconds();
            double dt = now - lastTime;
            lastTime = now;
            if( targetFwd != 0 ){
                // Calculate forward command for smooth deceleration using kinematics equation:
                double distFwd = targetFwd - localization.getX();
                double forward = Math.sqrt( 2 * ACCEL * distFwd );
                forward = Math.max( Math.min(forward, SPEED), -SPEED );
                // Use rate limiter to apply smooth acceleration:
                currentForward = applyRateLimit(forward, currentForward, ACCEL, dt);
            }
            else if( targetRight != 0 ){
                // Calculate forward command for smooth deceleration using kinematics equation:
                double distRight = targetRight - localization.getY();
                double right     = Math.sqrt( 2 * ACCEL * distRight );
                right = Math.max( Math.min(right, SPEED), -SPEED );
                // Use rate limiter to apply smooth acceleration:
                currentRight = applyRateLimit(right, currentRight, ACCEL, dt);
            }
            // Keep rotation on target:
            double rotate = localization.getHeadingDeg() * ANGLE_P;
            drive(currentForward, currentRight, rotate);
            telemetry.addData("Forward/Right/Rotate", "%.2f / %.2f / %.2f", currentForward, currentRight, rotate);
        }


        // Done! Now Auton_Basic.java will take over and disable drives and LED 
        return;
    }
}
