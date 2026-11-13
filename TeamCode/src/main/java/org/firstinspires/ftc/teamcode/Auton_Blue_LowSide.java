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
        while( opModeIsActive() && distances.WallDist_mm < Calibration.AUTON_STEP_1_DIST && runtime.seconds() < 3 ) {
            beginLoop("Step 1 (Move forward)");
            drive(0.3, 0, 0);
        }
        drive(0, 0, 0);

        // step 2 line up to April Tag while winding winch, waiting up to 23 seconds for it to tip our way
        while( opModeIsActive() && runtime.seconds() < 23 && !aprilSensing.OnTarget ) {
            beginLoop("Step 2 (Aimbot)");
            AlineToAprilTag();
        }

        // Step 3: Fire winch (timeout 2 seconds)
        launchCatapult();
        drive(0, 0, 0);
        while( opModeIsActive() && waitingForLaunch() ) {
            beginLoop("Step 3 (Launch)");
        }

        // Step 4: Strafe back to loading zone, while still winding winch
        while( opModeIsActive() && distances.getRight_mm() > 100 ){
            beginLoop("Step 4 (Strafe Right)");
            strafeAlongWall(Calibration.AUTON_STRAFE_SPEED);
        }

        // Step 5: Stop
        drive(0, 0, 0);

        telemetry.addData("Autonomy", "Complete");
        telemetry.update();
        sleep(1000);  // pause to display final telemetry message.
    }
}
