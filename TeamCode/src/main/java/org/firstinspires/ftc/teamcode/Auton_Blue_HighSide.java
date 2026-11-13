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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


@Autonomous(name="High Side: BLUE", group="Robot")
public class Auton_Blue_HighSide extends Auton_Basic {

    // Function that performs the desired autonomous tasks. Override this to create new autons
    // without repeating all the setup code.
    @Override
    public void doAuton()
    {
        // Step 1: move forward a few inches
        while(opModeIsActive() && distances.WallDist_mm < Calibration.AUTON_STEP_1_DIST && runtime.seconds() < 3) {
            beginLoop("Step 1 (Move forward)");
            drive(0.3, 0, 0);
        }
        drive(0, 0, 0);

        // step 2: line up to April Tag while we wait for hive to tip (or 25 seconds)
        while( opModeIsActive() && runtime.seconds() < 25 && !aprilSensing.OnTarget ) {
            beginLoop("Step 2 (Aimbot)");
            AlineToAprilTag();
        }
        drive(0, 0, 0);

        // Step 3: Fire winch (timeout 2 seconds)
        launchCatapult();
        while(opModeIsActive() && waitingForLaunch() ) {
            beginLoop("Step 3 (Launch)");
        }

        // Step 4: Strafe left until we're 1 meter from the wall
        while( opModeIsActive() && distances.getLeft_mm() > 1000 ){
            beginLoop("Step 4 (Strafe Left)");
            strafeAlongWall(-Calibration.AUTON_STRAFE_SPEED);
        }

        // Step 5: Rotate 90 degrees to the right, so dual distance sensors face the left wall
        imu.resetYaw();
        while( opModeIsActive() && distances.getIMUAngle() < 90 ){
            beginLoop("Step 5 (Turn Right)");
            drive(0, -Calibration.AUTON_STRAFE_SPEED, Calibration.AUTON_ROTATE_SPEED); // Rotate right while still strafing left
        }

        // Step 6: Strafe along left wall until we're 1 meter from the far wall
        while( opModeIsActive() && distances.getLeft_mm() > 1000 ){
            beginLoop("Step 6 (Strafe Left Again)");
            strafeAlongWall(-Calibration.AUTON_STRAFE_SPEED);
        }

        // Step 7: Stop
        drive(0, 0, 0);

        telemetry.addData("Autonomy", "Complete");
        telemetry.update();
        sleep(1000);  // pause to display final telemetry message.
    }
}
