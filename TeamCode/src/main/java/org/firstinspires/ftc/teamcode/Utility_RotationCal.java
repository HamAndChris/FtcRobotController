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


@Utility(name="Calibrate Rotation")
public class Utility_RotationCal extends Auton_Basic {

    public static final double ROTATE_RADPS  = 1.0; // commanded rotation speed

    // Function that performs the desired autonomous tasks. Override this to create new autons
    // without repeating all the setup code.
    @Override
    public void doAuton()
    {
        // Wait for user to press A before starting:
        while( opModeIsActive() && !gamepad1.a ) {
            beginLoop("Waiting for input");
            telemetry.addLine("Press A to begin moving.");
        }

        // Just apply a rotation command of 1 radians/sec and calculate the actual value
        double avgRot = 0;
        double coeff = 0.95;
        while(opModeIsActive()){
            beginLoop("Rotating");
            double rotate = localization.getRotateCommand(ROTATE_RADPS);
            drive(0, 0, rotate);

            // Apply IIR filter to IMU measured rotation:
            avgRot = avgRot * coeff + localization.getOmega() * (1 - coeff);
            telemetry.addData("Commanded", "%.2f", ROTATE_RADPS);
            telemetry.addData("Measured", "%.2f", avgRot);

            // Calculate what the rotation scale should be:
            double correct_multiplier = avgRot / ROTATE_RADPS * Localization.ROTATE_MULTIPLIER;
            telemetry.addData("Current ROTATE_MULTIPLIER", "%.4f", Localization.ROTATE_MULTIPLIER);
            telemetry.addData("True    ROTATE_MULTIPLIER", "%.4f", correct_multiplier);
        }


        // Done! Now Auton_Basic.java will take over and disable drives and LED 
        return;
    }
}
