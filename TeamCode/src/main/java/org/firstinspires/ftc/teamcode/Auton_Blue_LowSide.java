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

@Autonomous(name="Low Side: BLUE", group="Robot")
public class Auton_Blue_LowSide extends Auton_Basic {

    // Function that performs the desired autonomous tasks.
    @Override
    public void doAuton() 
    {
        // Step 1 move forward a few inches
        while( opModeIsActive() && distances.WallDist_inches < Calibration.AUTON_STEP_1_DIST && runtime.seconds() < 3 ) {
            beginLoop("Step 1 (Move forward)");
            drive(0.3, 0, 0);
        }
        drive(0, 0, 0);

        // step 2 line up to April Tag while winding winch, waiting up to 23 seconds for it to tip our way
        while( opModeIsActive() && runtime.seconds() < 23 ) {
            beginLoop("Step 2 (Aimbot)");
            AlineToAprilTag();
            if( aprilSensing.OnTarget ){
                // Successfully aligned to april tag. Fire away!
                launchCatapult();
                // Break out of loop:
                break;
            }
        }
        drive(0, 0, 0);
        setLEDTeamColor(); // Reset LED to team color after april alignment

        // Wait for catapult to finish launching (if it is)
        while( opModeIsActive() && waitingForLaunch() ) {
            beginLoop("Step 3 (Launch)");
        }

        // Step 4: Strafe back to loading zone, while winding winch.
        while( opModeIsActive() && distances.getRight_inches() > 16 ){
            beginLoop("Step 4 (Strafe Right)");
            strafeAlongWall(Calibration.AUTON_STRAFE_DIST, Calibration.AUTON_STRAFE_SPEED);
        }

        // Step 5: Rotate 90 left so our back is to the wall
        localization.resetYaw();
        while( opModeIsActive() && localization.getClockwiseAngle() > -90 ){
            beginLoop("Step 5 (Turn Left)");
            drive(0, 0, -Calibration.AUTON_ROTATE_SPEED);
        }

        // Step 6: backup to wall and hold wall distances in case we get bumped
        // But stop at 30 seconds to avoid penalties
        while( opModeIsActive() && runtime.seconds() < 30 ){
            beginLoop("Step 6 (Park)");
            holdPositionLeft(Calibration.AUTON_PARKING_REAR_DIST, 12);
        }

        // Done! Now Auton_Basic.java will take over and disable drives and LED
        return;
    }
}
