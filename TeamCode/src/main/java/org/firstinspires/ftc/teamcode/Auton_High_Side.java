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


@Autonomous(name="High Side", group="Robot")
public class Auton_High_Side extends Auton_Basic {

    @Override
    public String modeDescription(){
        return "Fire catapult once and return to parking zone";
    }

    @Override
    public String placementInstructions(){
        return( "Anywhere on right wall, facing hive." );
    }

    // Function that performs the desired autonomous tasks. Override this to create new autons
    // without repeating all the setup code.
    @Override
    public void doAuton()
    {
        // Set starting position as zero yaw so we can rotate after aiming:
        localization.resetYaw();

        // Step 1: move forward a few inches
        while(opModeIsActive() && distances.WallDist_inches < Calibration.AUTON_DIST_STEP1 && runtime.seconds() < 3) {
            beginLoop("Step 1 (Move forward)");
            drive(0.3, 0, 0);
        }
        drive(0, 0, 0);

        // step 2: line up to April Tag while we wait for hive to tip (or 25 seconds)
        while( opModeIsActive() && runtime.seconds() < 25 ) {
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

        // Step 3: Wait for catapult to finish launching
        while( opModeIsActive() && waitingForLaunch() ) {
            beginLoop("Step 3 (Launch)");
        }

        // Step 4: Rotate 90 degrees to the right, so dual distance sensors face the left wall
        while( opModeIsActive() && timesNotUp() && localization.getClockwiseAngle() < 90 ){
            beginLoop("Step 4 (Turn Right)");
            drive(0, -Calibration.AUTON_STRAFE_SPEED/2, Calibration.AUTON_ROTATE_MOTOR_SPEED); // Rotate right while still strafing left
        }

//        // Step 5: Strafe along driver wall until we're 1 meter from the left wall'
//        while( opModeIsActive() && timesNotUp() && distances.getLeft_mm() > 1000 ){
//            beginLoop("Step 5 (Strafe Left)");
//            strafeAlongWall(Calibration.AUTON_STRAFE_DIST, -Calibration.AUTON_STRAFE_SPEED);
//        }
//
//        // Step 7: backup to wall and hold wall distance in case we get bumped (but stop at 30 sec so no penalties)
//        while( opModeIsActive() && timesNotUp() ){
//            beginLoop("Step 7 (Park)");
//            strafeAlongWall(Calibration.AUTON_DIST_PARKING, 0);
//        }

        // Step 5: Strafe along driver wall until we hit something or are stopped (but stop at 30 sec so no penalties)
        while( opModeIsActive() && timesNotUp() ){
            beginLoop("Step 5 (Parking)");
            strafeAlongWall(Calibration.AUTON_DIST_PARKING, -Calibration.AUTON_STRAFE_SPEED);
        }

        // Done! Now Auton_Basic.java will take over and disable drives and LED
    }
}
