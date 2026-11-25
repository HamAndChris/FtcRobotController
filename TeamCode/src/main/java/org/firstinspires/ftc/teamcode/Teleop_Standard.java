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

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.ArrayDeque;
import java.util.Deque;
import java.util.Dictionary;
import java.util.Hashtable;
import java.util.List;

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
@TeleOp(name = "Teleop: Standard", group = "Robot")
//@Disabled
public class Teleop_Standard extends OpMode {
    // Hardware objects declared here:
    DcMotor frontLeftDrive;
    DcMotor frontRightDrive;
    DcMotor backLeftDrive;
    DcMotor backRightDrive;
    DcMotor winchMotor;
    DcMotor intakeMotor1;
    DcMotor intakeMotor2;
    DigitalChannel magnetSensor;
    Servo ledIndicator, flowerBar;

    // Helper files:
    private Distances         distances;
    private AprilSensing      aprilSensing;
    private Localization      localization;

    // Constants:

    // Winch state machine variables:
    private enum CatapultState { IDLE, WINDING, DOWN, LAUNCHING }
    private CatapultState catapultState = CatapultState.WINDING;
    private int winchLaunchStart;

    // FPS profiling:
    private final Deque<Double> timestamps = new ArrayDeque<>();
    private static final int FPS_SAMPLES = 3;
    private List<LynxModule> allHubs;
    private String modeString;

    // Team alliance color (now switchable during init):
    public String teamColor() {
        return redTeam ? "RED" : "BLUE";
    }
    private boolean redTeam = false, teamChosen = false;
    private static final String KEY_REDTEAM = "LastTeamRed";

    // Rate limiting variables:
    private double forwardLimited=0, rightLimited=0, rotateLimited=0, lastTime=0;

    // Other variables:
    private volatile Gamepad operatorGamepad; // Gamepad used for function buttons
    double flowerBarPosition = 0.5;
    Dictionary<HardwareDevice, Double> cachedMotorPowers = new Hashtable<>();
    double LEDColor;
    private double lastLaunchCommand = 0;  // Timestamp used for waitingForLaunch()
    private boolean lastRightStickButton = false;

    public double teamLEDColor(){
        return (teamColor().contains("BLUE")) ? Calibration.LED_BLUE : Calibration.LED_RED;
    }

    @Override
    public void init() {
        // Map hardware objects:
        intakeMotor1 = hardwareMap.get(DcMotor.class, "IntakeMotor");
        intakeMotor2 = hardwareMap.get(DcMotor.class, "IntakeMotor2");
        winchMotor = hardwareMap.get(DcMotor.class, "WinchMotor");
        magnetSensor = hardwareMap.get(DigitalChannel.class, "MagSensor");
        frontLeftDrive = hardwareMap.get(DcMotor.class, "FrontLeft");
        frontRightDrive = hardwareMap.get(DcMotor.class, "FrontRight");
        backLeftDrive = hardwareMap.get(DcMotor.class, "BackLeft");
        backRightDrive = hardwareMap.get(DcMotor.class, "BackRight");
        ledIndicator = hardwareMap.get(Servo.class, "Indicator");
        flowerBar    = hardwareMap.get(Servo.class, "FlowerBar");

        // We set the left motors in reverse which is needed for drive trains where the left
        // motors are opposite to the right ones.
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        winchMotor.setDirection(DcMotor.Direction.REVERSE);
        intakeMotor1.setDirection(DcMotor.Direction.REVERSE);
        intakeMotor2.setDirection(DcMotor.Direction.REVERSE);

        // Enable encoder velocity drive mode:
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Make motors brake when stopping
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        winchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        winchMotor.setPower(0);

        // Initialize the distance sensing algorithm:
        distances = new Distances(hardwareMap, telemetry);

        // Initialize the IMU and localization algorithm:
        localization = new Localization(hardwareMap, telemetry);

        // Initialize the April tag camera:
        aprilSensing = new AprilSensing(teamColor(), hardwareMap, telemetry, localization);

        // Set the control and expansion hub to "bulk read" mode to improve performance
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        // Decide if we're using one gamepad or two:
        if( Calibration.SINGLE_GAMEPAD_MODE ){
            operatorGamepad = gamepad1;
            modeString = "Single Gamepad Mode";
        } else {
            operatorGamepad = gamepad2;
            modeString = "Dual Gamepad Mode";
        }

        // If we've already set the team color, allow user to confirm. Otherwise, encourage them to choose.
        Boolean prevRedTeam = (Boolean) blackboard.get(KEY_REDTEAM);
        if( prevRedTeam != null ){
            teamChosen = true;
            redTeam = prevRedTeam;
            aprilSensing.setTeamColor(teamColor());
        }
    }

    @Override
    public void init_loop() {
        // Clear cached hardware reads to speed up I/O access:
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }
        displayFPS(modeString);
        distances.update();
        localization.update();

        // Keep the winch wound when the user initialized the robot. Actually don't, that can be a penalty.
        //doWinchStatemachine();

        // Allow user to set alliance team, or use previously set value
        telemetry.addLine();
        telemetry.addLine("*****************************************");
        telemetry.addLine("*   Select Team with Driver Gamepad");
        if (teamChosen) {
            telemetry.addData("*        Selected", teamColor());
        }
        telemetry.addLine("*");
        telemetry.addLine("*             ( X )                  ( B )");
        telemetry.addLine("*            BLUE                 RED");
        telemetry.addLine("*****************************************");

        if( teamChosen ){
            setLEDTeamColor();
        }
        else {
            // Blink LED while we wait for user to select a team color with the gamepad.
            double partial = time % 0.8;
            setLED( (partial > 0.4) ? Calibration.LED_RED : Calibration.LED_BLUE );
        }

        // Accept driver gamepad input to select the color
        if (gamepad1.b) {
            // B button is red so it sets red team
            redTeam = true;
            teamChosen = true;
            aprilSensing.setTeamColor(teamColor());
            blackboard.put(KEY_REDTEAM, true); // Save in case we forget to set it next time
        } else if (gamepad1.x) {
            // X button is blue so it sets blue team
            redTeam = false;
            teamChosen = true;
            aprilSensing.setTeamColor(teamColor());
            blackboard.put(KEY_REDTEAM, false); // Save in case we forget to set it next time
        }
        telemetry.update();
    }

    @Override
    public void loop() {

        // Clear cached hardware reads to speed up I/O access:
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }

        displayFPS(modeString);

        // Read angle sensors and IMU:
        distances.update();
        localization.update();

        /* *************** LED Team Color / Blinking Logic ************* */

        // Blink LED for our team alliance, and blink faster when there's 10 seconds left
        // Note: This may be overwritten by AlineToAprilTag
        double partial=(time-Math.floor(time));
        if( time < 112
                || (time < 116 && partial > 0.5)
                || (time < 120 && partial % 0.5 > 0.25)
                || (time > 125) ) {
            LEDColor = teamLEDColor();
        }
        else {
            LEDColor = Calibration.LED_OFF;
        }

        /* *************** Yaw orientation reset logic ************* */

        if( gamepad1.right_stick_button ){
            // Right stick button is pressed. Treat the robot facing away from driver as "forward" so it must be rotated 90 degrees
            localization.setHeadingDeg(90);
        }

        /* *************** Chassis Movement Controls *************** */

        // Get chassis stick inputs, applying parabolic transform:
        double forward = -(gamepad1.left_stick_y * Math.abs(gamepad1.left_stick_y)) * Calibration.TELEOP_POWER_LEVEL;
        double right = gamepad1.left_stick_x * Math.abs(gamepad1.left_stick_x) * Calibration.TELEOP_POWER_LEVEL;
        double rotate = gamepad1.right_stick_x * Math.abs(gamepad1.right_stick_x) * Calibration.TELEOP_POWER_LEVEL;

        // Apply field relative conversion:
        if( Calibration.USE_FIELD_RELATIVE ){
            localization.convertFieldRelative(forward, right, rotate);
            forward = localization.newForward;
            right = localization.newRight;
            rotate = localization.newRotate;
        }

        // Decide how chassis should be controlled:
        if( waitingForLaunch() ){
            // When winch is launching, DO NOT MOVE THE ROBOT!
            drive(0, 0, 0);
        }
        else if (gamepad1.left_bumper) {
            // Left bumper is now APRIL tag AUTO AIMBOT MODE.
            // Pressing BOTH BUMPERS will actually align. Left only will just display on telemetry
            AlineToAprilTag( !gamepad1.right_bumper, forward, right, rotate );
        } 
        else if (gamepad1.right_bumper) {
            // Right bumper enables wall following strafe mode
            strafeAlongWall(right);
        } 
        else {
            // No bumpers being pressed. Apply gamepad inputs commands:
            drive(forward, right, rotate);
        }

        /* ************* Winch State Machine *********** */

        doWinchStatemachine();

        /* ************* Intake Motor Logic *********** */

        // Left trigger spits out, even if right trigger is still held:
        if( operatorGamepad.left_trigger > 0.2 ){
            setIntakeOut();
        }
        else if( operatorGamepad.right_trigger > 0.2 && catapultState == CatapultState.DOWN ){
            intakeOn();
        }
        else {
            intakeOff();
        }

        /* ************* Flower Bar Controls *********** */

        if( operatorGamepad.dpad_up ){
            flowerBarUp();
        }
        if( operatorGamepad.dpad_down ){
            flowerBarDown();
        }

        // Send LED value:
        setPositionCached(ledIndicator, LEDColor);

        // Send all telemetry to driver station:
        telemetry.update();

        // Track previous loop time for rate calculations:
        lastTime = time;
    }

    /** Helper function to set Flower Bar to down position */
    void flowerBarDown(){
        setPositionCached(flowerBar, Calibration.FLOWER_BAR_DOWN);
    }
    /** Helper function to set Flower Bar to up position */
    void flowerBarUp(){
        setPositionCached(flowerBar, Calibration.FLOWER_BAR_UP);
    }

    /** Helper to set both intake motors to a specific speed */
    private void setIntakeMotors(double speed){
        setPowerCached(intakeMotor1, speed);
        setPowerCached(intakeMotor2, speed);
    }

    /** Helper function to set intake motors on */
    void intakeOn(){
        setIntakeMotors(1.0);
    }
    /** Helper function to set intake motors off */
    void intakeOff(){
        setIntakeMotors(0.0);
    }
    /** Helper function to set intake motors in reverse */
    void setIntakeOut(){
        setIntakeMotors(-1.0);
    }

    /** Helper function to set LED color. Try not to change more than once per loop. */
    void setLED(double color){
        setPositionCached(ledIndicator, color);
    }
    /** Helper function to set LED color to match configured team. Try not to change more than once per loop. */
    void setLEDTeamColor(){
        setLED( (teamColor().contains("BLUE")) ? Calibration.LED_BLUE : Calibration.LED_RED );
    }

    // Record a call and report telemetry the calculated calls-per-second over the last few calls.
    // Make sure you always call this at the beginning of each loop in doAuton()
    void displayFPS(String label) {

        // Get current timestamp:
        double now = time;
        double fps = 0.0;
        timestamps.addLast(now);

        // Keep only the last maxSamples timestamps
        if (timestamps.size() > FPS_SAMPLES) {
            timestamps.removeFirst();
        }

        // Calculate FPS if there are enough samples
        if (timestamps.size() == FPS_SAMPLES) {
            // Calculate rate: (number of calls - 1) / time span
            double span = timestamps.getLast() - timestamps.getFirst();
            fps = span > 0 ? (timestamps.size() - 1) / span : 0.0;
        }

        telemetry.addData(label, "%.2f s, %.1f FPS", now, fps);
    }

    private void doWinchStatemachine() {
        // Idle state:
        if( catapultState == CatapultState.IDLE ){
            // Turn motor off
           if (operatorGamepad.b ){
               setPowerCached(winchMotor, -Calibration.WINCH_UNWIND_POWER);
           }
           else {
               setPowerCached(winchMotor, 0);
           }
            // Check conditions
            if( operatorGamepad.y ){
                catapultState = CatapultState.WINDING;
            }
        }
        // Winding state
        else if( catapultState == CatapultState.WINDING ){
            // Turn motor on
            setPowerCached(winchMotor, 1);
            // Check conditions
            if( !magnetSensor.getState() ){
                catapultState = CatapultState.DOWN;
                winchLaunchStart = winchMotor.getCurrentPosition();
            }
            if( operatorGamepad.b ){
                catapultState = CatapultState.IDLE;
            }
        }
        // Down state
        else if( catapultState == CatapultState.DOWN ){
            // Apply holding torque to motor
            double winchPower = (winchMotor.getCurrentPosition()-winchLaunchStart)
                    * -Calibration.WINCH_HOLDING_GAIN_P + Calibration.WINCH_HOLDING_FEEDFORWARD;
            setPowerCached(winchMotor, winchPower);

            // Check conditions
            if( operatorGamepad.x ){
                catapultState = CatapultState.LAUNCHING;
                lastLaunchCommand = time;
            }
            if( operatorGamepad.b ){
                catapultState = CatapultState.IDLE;
            }
        }
        // Launching state
        else if( catapultState == CatapultState.LAUNCHING ){
            // Turn motor on
            setPowerCached(winchMotor, 1);
            // Check conditions
            if( (winchMotor.getCurrentPosition() - winchLaunchStart) > Calibration.WINCH_LAUNCH_AMOUNT ){
                catapultState = CatapultState.WINDING; // Go back to winding unless canceled
            }
            if( operatorGamepad.b ){
                catapultState = CatapultState.IDLE;
            }
        }
        else {
            catapultState = CatapultState.IDLE;
        }
    }

    /** Return true if the catapult has not finished launching or timed out */
    boolean waitingForLaunch(){
        return catapultState == CatapultState.LAUNCHING && (time - lastLaunchCommand) < Calibration.WINCH_FIRING_LOCKOUT;
    }

    // Helper function to speed up loop time
    void setPowerCached(DcMotor motor, double power){
        // Get previous power value:
        Double prevPower = cachedMotorPowers.get(motor);
        if( prevPower == null ){
            prevPower = 0.0;
        }
        // Only update motor power if it changed by more than 1%, or stopping:
        if( Math.abs(power - prevPower) > 0.01 || (power == 0.0 && prevPower != 0.0) ) {
            motor.setPower(power);
            cachedMotorPowers.put(motor, power);
        }
    }

    // Helper function to speed up loop time
    void setPositionCached(Servo servo, double pos){
        // Get previous power value:
        Double prevPos = cachedMotorPowers.get(servo);
        if( prevPos == null ){
            prevPos = 0.0;
        }
        // Only update position if it changed by more than 2%:
        if( Math.abs(pos - prevPos) > 0.02 ) {
            servo.setPosition(pos);
            cachedMotorPowers.put(servo, pos);
        }
    }

    // Use the AprilSensing class to detect and choose alignment commands, then apply them along with the LED color:
    void AlineToAprilTag(boolean display_only, double forward, double right, double rotate)
    {
        // Perform the sensing
        aprilSensing.performSensing(forward, right, rotate);
        if( display_only ){
            // Display only mode, pass through original commands.
            drive(forward, right, rotate);
        }
        else {
            // Alignment mode. Pass through the alignment algorithm:
            drive(  aprilSensing.Forward,
                    aprilSensing.Right,
                    aprilSensing.Rotate);
        }

        // Turn on LED:
        if( aprilSensing.OnTarget ){
            LEDColor = Calibration.LED_READY_TO_FIRE;
        }
        else if( aprilSensing.Detected ){
            LEDColor = Calibration.LED_AIMING;
        }
        else {
            LEDColor = teamLEDColor();
        }
    }


    /**
     * Checks if a gamepad is connected based on its ID.
     */
    private boolean isGamepadConnected(Gamepad gp) {
        return gp != null && gp.id != Gamepad.ID_UNASSOCIATED;
    }

    // Apply motor torques to follow the wall at the desired strafing speed
    void strafeAlongWall(double right)
    {
        // Calculate angle P control loop:
        double rotatePID = (distances.WallAngle_Deg * -Calibration.LASER_ANGLE_GAIN_P);
        rotatePID = Math.min(Math.max(rotatePID, -Calibration.LASER_ANGLE_MAXPOWER), Calibration.LASER_ANGLE_MAXPOWER); // Limit to maximum rotation command

        // Calculate distance P control loop:
        double forwardPID = (Calibration.LASER_TARGET_DIST_MM - distances.WallDist_mm) * Calibration.LASER_DIST_GAIN_P;
        forwardPID = Math.max(Math.min(forwardPID, Calibration.LASER_DIST_MAXPOWER), -Calibration.LASER_DIST_MAXPOWER); // Limit to maximum forward command

        // Correct forward/right ratio with trigonometry to keep moving straight along the wall:
        double rightCorrected = right * Math.cos(distances.WallAngle_Rads) - forwardPID * Math.sin(distances.WallAngle_Rads);
        double forwardCorrected = forwardPID * Math.cos(distances.WallAngle_Rads) + right * Math.sin(distances.WallAngle_Rads);

        drive(forwardCorrected, rightCorrected, rotatePID);
        telemetry.addData("Forward PID", "%.2f", forwardPID);
        telemetry.addData("Rotate PID", "%.2f", rotatePID);
    }

    // This routine drives the robot field relative
    private void driveFieldRelative(double forward, double right, double rotate) {
        // First, convert direction being asked to drive to polar coordinates
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(right, forward);

        // Second, rotate angle by the angle the robot is pointing
        theta = AngleUnit.normalizeRadians(theta - localization.getYawRadians());

        // Third, convert back to Cartesian
        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);

        // Finally, call the drive method with robot relative forward and right amounts
        drive(newForward, newRight, rotate);
    }

    // Thanks to FTC16072 for sharing this code!!
    public void drive(double forward, double right, double rotate)
    {
        double maxPower = 1;
        double maxSpeed = 1;  // make this slower for outreaches


        // Apply rate limits to smooth out robot motion:
//        forwardLimited = applyRateLimit(forward, forwardLimited, Calibration.STICK_RATE_LIMIT);
//        rightLimited   = applyRateLimit(right,   rightLimited,   Calibration.STICK_RATE_LIMIT);
//        rotateLimited  = applyRateLimit(rotate,  rotateLimited,  Calibration.STICK_RATE_LIMIT);
        forwardLimited = forward;
        rightLimited = right;
        rotateLimited = rotate;

        // This calculates the power needed for each wheel based on the amount of forward,
        // strafe right, and rotate
        double frontLeftPower = forwardLimited + rightLimited + rotateLimited;
        double frontRightPower = forwardLimited - rightLimited - rotateLimited;
        double backRightPower = forwardLimited + rightLimited - rotateLimited;
        double backLeftPower = forwardLimited - rightLimited + rotateLimited;


        // This is needed to make sure we don't pass > 1.0 to any wheel
        // It allows us to keep all the motors in proportion to what they should
        // be and not get clipped
        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));

        // We multiply by maxSpeed so that it can be set lower for outreaches
        // When a young child is driving the robot, we may not want to allow full
        // speed.
        setPowerCached(frontLeftDrive, maxSpeed * (frontLeftPower / maxPower));
        setPowerCached(frontRightDrive, maxSpeed * (frontRightPower / maxPower));
        setPowerCached(backLeftDrive, maxSpeed * (backLeftPower / maxPower));
        setPowerCached(backRightDrive, maxSpeed * (backRightPower / maxPower));
    }
}
