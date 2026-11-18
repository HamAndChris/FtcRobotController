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

import com.qualcomm.robotcore.eventloop.opmode.Utility;


@Utility(name="Test Odometry")
public class Utility_TestOdometry extends Auton_Basic {

    public static final double DISTANCE  = 24; // inches
    public static final double ANGLE     = 90; // degrees

    // Driver info:

    @Override
    public String modeDescription(){
        return "Test the various autonomous driving functions";
    }

    @Override
    public String placementInstructions(){
        return( String.format("Somewhere with at least %.1f inches of space", DISTANCE*2) );
    }

    // Function that performs the desired autonomous tasks. Override this to create new autons
    // without repeating all the setup code.
    @Override
    public void doAuton()
    {
        localization.setPose(0,0,0);
        double targetX = 0, targetY = 0, targetHeading = 0;

        // Wait for user to press the D-pad direction they wish to go:
        while( opModeIsActive() ){
            beginLoop("Waiting for input");
            telemetry.addData("Press D-Pad", "translate %.1f inches", DISTANCE);
            telemetry.addData("Press Bumpers", "rotate %.1f degrees", ANGLE);
            if( gamepad1.dpad_up ){
                targetX += DISTANCE;
                autoSlideTo(targetX, targetY, targetHeading);
            }
            else if( gamepad1.dpad_down ){
                targetX -= DISTANCE;
                autoSlideTo(targetX, targetY, targetHeading);
            }
            else if( gamepad1.dpad_right ){
                targetY += DISTANCE;
                autoSlideTo(targetX, targetY, targetHeading);
            }
            else if( gamepad1.dpad_left ){
                targetY -= DISTANCE;
                autoSlideTo(targetX, targetY, targetHeading);
            }
            else if( gamepad1.left_bumper ){
                targetHeading += ANGLE;
                autoRotateTo(targetHeading);
            }
            else if( gamepad1.right_bumper ){
                targetHeading -= ANGLE;
                autoRotateTo(targetHeading);
            }
        }

        // Done! Now Auton_Basic.java will take over and disable drives and LED
    }
}
