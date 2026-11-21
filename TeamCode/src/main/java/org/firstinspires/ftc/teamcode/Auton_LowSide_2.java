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

@Autonomous(name="Low Side 2.0", group="Robot")
public class Auton_LowSide_2 extends Auton_Basic {

    // Starting position and heading:

    /** Inches: Center of robot to left wall */
    public static final double STARTING_X       = 9;
    /** Inches: Center of robot to driver wall */
    public static final double STARTING_Y       = 9 + 52;
    /** Degrees: Counterclockwise rotation from facing right wall */
    public static final double STARTING_HEADING = 0;

    // Driver info:

    @Override
    public String modeDescription(){
        return "Wait for hive tip, fire catapult, drain flower, fire again, park";
    }

    @Override
    public String placementInstructions(){
        return( "Low Side wall, two inches left of Flower" );
    }

    // Function that performs the desired autonomous tasks:
    @Override
    public void doAuton() 
    {
        // Set starting position:
        localization.setPose(STARTING_X, STARTING_Y, STARTING_HEADING);

        // Step 1 move forward a few inches
        autoSlideToAbout(STARTING_X + Calibration.AUTON_DIST_STEP1, STARTING_Y, STARTING_HEADING);

        // step 2 line up to April Tag while winding winch, wait up to 23 seconds for it to tip our way
        autoAlignAndFire(25);

        // If we have time, try to drain the flower and shoot again
        if( runtime.seconds() < 15 ) {
            // step 3 turn around and approach flower
            autoSlideToLeftFlower();

            // step 4: Turn on intake and drain flower
            autoDrainFlower();

            // step 5: Turn back around towards hive
            autoSlideToAbout(Poses.LEFT_FIRING);
            intakeOff();

            // step 6: try to fire again
            autoAlignAndFire(27);
        }

        // Slide over towards parking zone and rotate 90 so back faces driver wall:
        autoSlideToAbout(21, 24, 90);

        // Step 6: backup to wall and hold wall distances in case we get bumped
        // But stop at 30 seconds to avoid penalties
        autoHoldDistances(Calibration.AUTON_DIST_PARKING_REAR, Calibration.AUTON_DIST_PARKING_LEFT1);

        // Done! Now Auton_Basic.java will take over and disable drives and LED
    }
}
