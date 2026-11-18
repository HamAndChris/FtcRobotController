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


@Autonomous(name="High Side 2.0", group="Robot")
public class Auton_HighSide_2 extends Auton_Basic {

    // Starting position and heading:
    /** Inches: Center of robot to left wall */
    public static final double STARTING_X       = 135;
    /** Inches: Center of robot to driver wall */
    public static final double STARTING_Y       = 9 + 52;
    /** Degrees: Counterclockwise rotation from facing right wall */
    public static final double STARTING_HEADING = 0;

    @Override
    public String modeDescription(){
        return "Fire catapult, drain flower, and return to parking zone";
    }

    @Override
    public String placementInstructions(){
        return( "Flat against right wall, directly facing hive." );
    }

    // Function that performs the desired autonomous tasks. Override this to create new autons
    // without repeating all the setup code.
    @Override
    public void doAuton()
    {
        // Set starting position:
        localization.setPose(STARTING_X, STARTING_Y, STARTING_HEADING);

        // Step 1 move forward a few inches
        autoSlideTo(STARTING_X + Calibration.AUTON_DIST_STEP1, STARTING_Y);

        // step 2 line up to April Tag while winding winch, wait up to 23 seconds for it to tip our way
        autoAlignAndFire(25);

        // Step 4: Strafe left from current wall distance until we're 1 meter from the wall
        double startingWallDist = distances.WallDist_inches;
        while( opModeIsActive() && distances.getLeft_mm() > 1000 ){
            beginLoop("Step 4 (Strafe Left)");
            strafeAlongWall(startingWallDist, -Calibration.AUTON_STRAFE_SPEED);
        }

        // Step 5: Rotate 90 degrees to the right, so dual distance sensors face the left wall
        localization.resetYaw();
        while( opModeIsActive() && localization.getClockwiseAngle() < 90 ){
            beginLoop("Step 5 (Turn Right)");
            drive(0, -Calibration.AUTON_STRAFE_SPEED/2, Calibration.AUTON_ROTATE_MOTOR_SPEED); // Rotate right while still strafing left
        }

        // Step 6: Strafe along left wall until we're 1 meter from the far wall
        while( opModeIsActive() && timesNotUp() && distances.getLeft_mm() > 1000 ){
            beginLoop("Step 6 (Strafe Left Again)");
            strafeAlongWall(Calibration.AUTON_STRAFE_DIST, -Calibration.AUTON_STRAFE_SPEED);
        }

        // Step 7: backup to wall and hold wall distance in case we get bumped (but stop at 30 sec so no penalties)
        while( opModeIsActive() && timesNotUp() ){
            beginLoop("Step 7 (Park)");
            strafeAlongWall(Calibration.AUTON_DIST_PARKING, 0);
        }

        // Done! Now Auton_Basic.java will take over and disable drives and LED 
        return;
    }
}
