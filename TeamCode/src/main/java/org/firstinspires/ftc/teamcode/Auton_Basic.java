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

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayDeque;
import java.util.Deque;
import java.util.Dictionary;
import java.util.Hashtable;
import java.util.List;


/* This OpMode will be the basis for all other Autonomous OpModes.
 * 
 * We will initialize all the inputs and outputs, but then just drive forward three inches.
 * Other Autonomous modes can extend this class and override the "doAuton" function.
*/

@Autonomous(name="Basic (Slide Right)", group="Robot")
public class Auton_Basic extends LinearOpMode {

    // Hardware objects:
    DcMotor         frontLeftDrive   = null;
    DcMotor         frontRightDrive   = null;
    DcMotor         backLeftDrive   = null;
    DcMotor         backRightDrive   = null;
    DcMotor         intakeMotor1;
    DcMotor         intakeMotor2;
    DcMotor         winchMotor;
    AnalogInput     laserAnalog1, laserAnalog2;
    DistanceSensor  distanceRight, distanceLeft;
    DigitalChannel  magnetSensor;
    Servo           ledIndicator = null;
    Servo           flowerBar = null;
    IMU             imu;
    VisionPortal    visionPortal;
    private List<LynxModule> allHubs;

    // Helper files:
    AprilTagProcessor aprilTag;
    AprilSensing      aprilSensing;
    Distances         distances;
    Localization      localization;

    // Winch state machine variables:
    enum CatapultState { IDLE, WINDING, DOWN, LAUNCHING }
    int           winchLaunchStart;
    CatapultState catapultState = CatapultState.WINDING; // Always start winding

    // Other variables:
    final ElapsedTime runtime = new ElapsedTime();
    Dictionary<HardwareDevice, Double> cachedMotorPowers = new Hashtable<>(); // For performance improvement
    private double lastLaunchCommand = 0;  // Timestamp used for launchCatapult()
    private double lastMisalignedTime = 0; // Timestamp used for rotateToIMUAngle()

    // FPS profiling:
    private final Deque<Double> timestamps = new ArrayDeque<>();
    private static final int FPS_SAMPLES = 3;

    // Default team color:
    public String teamColor() {
        return redTeam ? "RED" : "BLUE";
    }
    private boolean redTeam = false; // Now switchable

    // Helper to stop all "auto loop" auton modes once time is up:
    boolean timesNotUp(){
        return runtime.seconds() < Calibration.AUTON_TIME_LIMIT;
    }

    /** Placement instructions to show on Driver Hub during initialization. Other auton modes should override this. */
    public String placementInstructions(){
        return( "On driver wall, left of parking zone" );
    }

    /** Description to show on Driver Hub during initialization. Other auton modes should override this. */
    public String modeDescription(){
        return String.format("Moves forward %.1f inches, right %.1f inches", Calibration.AUTON_DIST_STEP1, 24.0);
    }

    @Override
    public void runOpMode() {
        // 1. Retrieve all connected REV Hubs (Control Hub + Expansion Hub)
        allHubs = hardwareMap.getAll(LynxModule.class);
        // 2. Set every hub to MANUAL bulk caching mode
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        frontLeftDrive = hardwareMap.get(DcMotor.class, "FrontLeft");
        frontRightDrive = hardwareMap.get(DcMotor.class, "FrontRight");
        backLeftDrive = hardwareMap.get(DcMotor.class, "BackLeft");
        backRightDrive = hardwareMap.get(DcMotor.class, "BackRight");
        magnetSensor = hardwareMap.get(DigitalChannel.class, "MagSensor");
        intakeMotor1 = hardwareMap.get(DcMotor.class, "IntakeMotor");
        intakeMotor2 = hardwareMap.get(DcMotor.class, "IntakeMotor2");
        winchMotor = hardwareMap.get(DcMotor.class, "WinchMotor");
        ledIndicator = hardwareMap.get(Servo.class, "Indicator");
        flowerBar    = hardwareMap.get(Servo.class, "FlowerBar");

        // We set the left motors in reverse which is needed for drive trains where the left
        // motors are opposite to the right ones.
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        winchMotor.setDirection(DcMotor.Direction.REVERSE);
        intakeMotor1.setDirection(DcMotor.Direction.REVERSE);
        intakeMotor2.setDirection(DcMotor.Direction.REVERSE);

        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Make motors brake when stopping
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        winchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize the distance sensing algorithm:
        distances = new Distances(hardwareMap, telemetry);

        // Initialize the IMU and localization algorithm:
        localization = new Localization(hardwareMap, telemetry);

        // Initialize the APRIL tag sensing / webcam
        aprilSensing = new AprilSensing(teamColor(), hardwareMap, telemetry, localization);


        /* ********** Keep the winch wound while we wait for user to press START ************ */

        while( opModeInInit() ){
            beginLoop("Choose a Team");
            telemetry.addLine();
            telemetry.addLine("****************************");
            telemetry.addLine("*");
            telemetry.addLine("*  ( X ) = BLUE team");
            telemetry.addLine("*");
            telemetry.addLine("*             RED team = ( B )");
            telemetry.addLine("*");
            telemetry.addLine("****************************");
            telemetry.addLine();
            telemetry.addData("Description", modeDescription());
            telemetry.addData("Placement", placementInstructions());

            // Blink red/blue while we wait for user to select a team color with the gamepad.
            // If they press Start without choosing one, it'll be whatever color was lit when they pressed it.
            double partial = runtime.seconds() % 0.8;
            redTeam = (partial > 0.4);
            setLEDTeamColor();
            if( gamepad1.b ){
                redTeam = true; // B button is red so it sets red team
                break;
            }
            else if( gamepad1.x ){
                redTeam = false; // X button is blue so it sets blue team
                break;
            }
        }

        // Set the LED to team color, and update the April Sensing logic:
        setLEDTeamColor();
        aprilSensing.setTeamColor(teamColor());

        // Now just wait for START button from driver hub
        while( opModeInInit() ){
            beginLoop("Waiting for Start");
            telemetry.addLine();
            telemetry.addData("Team chosen", teamColor() );
            telemetry.addLine();
            telemetry.addData("Description", modeDescription());
            telemetry.addData("Placement", placementInstructions());
        }


        /* *********** Call doAuton, which will be overridden by other auton modes ********** */

        runtime.reset(); // Set runtime to zero for timing
        doAuton();


        /* **************************** Wrap everything up.  ************************** */

        // After all steps complete, make sure to stop and turn off the LED:
        drive(0, 0, 0);
        setLED(Calibration.LED_OFF);

        telemetry.addData("Autonomy", "Complete");
        telemetry.update();
        sleep(1000);  // pause to display final telemetry message.
    }

    // Function that performs the desired autonomous tasks. Override this to create new autons
    // without repeating all the setup code.
    public void doAuton()
    {
        // Step 1 move forward 6 inches
        while( opModeIsActive() && distances.WallDist_inches < Calibration.AUTON_DIST_PARKING && runtime.seconds() < 3 ) {
            beginLoop("Step 1 (Moving Forward)");
            drive(0.2, 0, 0);
        }
        
        // Step 2 slide right until we're 24 inches from the wall, then hold there so we don't get pushed out of the box
        // But stop at 30 seconds to avoid penalties
        while( opModeIsActive() && runtime.seconds() < 30 ) {
            beginLoop("Step 2 (Parking)");
            holdPositionLeft(Calibration.AUTON_DIST_PARKING, 24);
        }

        // Done! Now runOpMode() will take over and disable drives and LED 
        return;
    }

    // Record a call and report telemetry the calculated calls-per-second over the last few calls.
    // Make sure you always call this at the beginning of each loop in doAuton()
    void beginLoop(String label) {
        // Send the previously added telemetry lines:
        telemetry.update();

        // Clear cached hardware reads to speed up I/O access:
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }

        // Get current timestamp:
        double now = runtime.seconds();
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

        // Add first lines of telemetry:
        telemetry.addData(label, "%s, %.2f s, %.1f FPS", teamColor(), now, fps);

        // Update the odometry and laser sensors here since we always want the moving averages running:
        distances.update();
        localization.update();

        // Also process the winch state machine since we always want to keep its control loop active:
        doWinchStatemachine(false, false);
    }

    // Helper function to set LED color
    void setLED(double color){
        setPositionCached(ledIndicator, color);
    }
    void setLEDTeamColor(){
        setLED( (teamColor().contains("BLUE")) ? Calibration.LED_BLUE : Calibration.LED_RED );
    }


    // Helper to rotate the robot to a desired angle (positive = RIGHT), while optionally strafing and driving.
    // Returns true when angle has been achieved.
    // NOTE: IMU reading wraps around at 180 degrees, so never exceed that and always call imu.reset() beforehand
    boolean rotateToIMUAngle(double targetAngle, double right, double forward)
    {
        double currentAngle = localization.getClockwiseAngle();
        // Apply rotation control loop:
        double rotate = (targetAngle - currentAngle) * Calibration.AUTON_ANGLE_GAIN_P;
        drive(forward, right, rotate);

        // Return true if the angle is within 2 degrees for 0.5 seconds:
        double now = runtime.seconds();
        if( Math.abs(targetAngle - currentAngle) > Calibration.AUTON_ANGLE_THRESH ){
            lastMisalignedTime = now;
        }
        return (now - lastMisalignedTime) >= Calibration.AUTON_ANGLE_SETTLING;
    }


    // Apply motor torques to follow the wall at the desired strafing speed
    void strafeAlongWall(double wallDist_inches, double rightSpeed)
    {
        // Apply rotation control loop:
        double rotate = distances.WallAngle_Deg * -Calibration.LASER_ANGLE_GAIN_P;
        rotate = Math.min(Math.max(rotate, -Calibration.LASER_ANGLE_MAXPOWER), Calibration.LASER_ANGLE_MAXPOWER); // Limit to maximum rotation command

        // Apply distance control loop:
        double forward = (wallDist_inches*25.4 - distances.WallDist_mm) * Calibration.LASER_DIST_GAIN_P;
        forward = Math.min(Math.max(forward, -Calibration.LASER_DIST_MAXPOWER), Calibration.LASER_DIST_MAXPOWER); // Limit to maximum forward command

        // Correct forward/right ratio to keep moving straight along wall:
        double newRight = rightSpeed * Math.cos(distances.WallAngle_Rads) - forward * Math.sin(distances.WallAngle_Rads);
        double newForward = forward * Math.cos(distances.WallAngle_Rads) + rightSpeed * Math.sin(distances.WallAngle_Rads);
        drive(newForward, newRight, rotate);
    }

    // Apply motor torques to hold the desired distance from rear wall and left distance sensor
    void holdPositionLeft(double wallDist_inches, double leftDist_inches)
    {
        double right = (leftDist_inches*25.4 - distances.getLeft_mm()) * Calibration.LASER_DIST_GAIN_P;
        right = Math.min(Math.max(right, -Calibration.LASER_DIST_MAXPOWER), Calibration.LASER_DIST_MAXPOWER); // Limit to max left/right command

        strafeAlongWall(wallDist_inches, right);
    }

    // Use the AprilSensing class to detect and choose alignment commands, then apply them along with the LED color:
    void AlineToAprilTag() {
        aprilSensing.performSensing(0, 0, 0);
        drive(  aprilSensing.Forward,
                aprilSensing.Right,
                aprilSensing.Rotate);
        if( aprilSensing.OnTarget ){
            setPositionCached(ledIndicator, Calibration.LED_READY_TO_FIRE);
        }
        else if( aprilSensing.Detected ){
            setPositionCached(ledIndicator, Calibration.LED_AIMING);
        }
    }
    
    // Drive both intake motors (1.0 full intake, -1.0 full reverse)
    void setIntakeMotors(double speed){
        setPowerCached(intakeMotor1, speed);
        setPowerCached(intakeMotor2, speed);
    }

    /** Helper function to set Flower Bar to down position */
    void flowerBarDown(){
        setPositionCached(flowerBar, Calibration.FLOWER_BAR_DOWN);
    }
    /** Helper function to set Flower Bar to up position */
    void flowerBarUp(){
        setPositionCached(flowerBar, Calibration.FLOWER_BAR_UP);
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

    /** Call this to trigger a catapult launch, but not wait for results */
    void launchCatapult(){
        // Manipulating the state directly. Only allow when in DOWN state.
        if( catapultState == CatapultState.DOWN ){
            catapultState = CatapultState.LAUNCHING;
            lastLaunchCommand = runtime.seconds();
        }
    }

    /** Return true if the catapult has finished launching or timed out */
    boolean waitingForLaunch(){
        return catapultState == CatapultState.LAUNCHING && (runtime.seconds() - lastLaunchCommand) < 2.0;
    }
    
    // Simplified version of the teleop winch state machine; no unwinding, or recovery from IDLE
    void doWinchStatemachine(boolean fireCatapult, boolean goToIdle) {
        // Idle state:
        if( catapultState == CatapultState.IDLE ){
            // Turn motor off
            setPowerCached(winchMotor, 0);

            // No conditions to exit IDLE in autonomous mode :(
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
            if( goToIdle ){
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
            if( fireCatapult ){
                catapultState = CatapultState.LAUNCHING;
            }
            if( goToIdle ){
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
            if( goToIdle ){
                catapultState = CatapultState.IDLE;
            }
        }
        else {
            catapultState = CatapultState.IDLE;
        }

        telemetry.addData("Winch State", catapultState.name());
    }

    // Helper function to speed up loop time
    void setPowerCached(DcMotor motor, double power){
        // Get previous power value:
        Double prevPower = cachedMotorPowers.get(motor);
        if( prevPower == null ){
            prevPower = 0.0;
        }
        // Only update motor power if it changed by more than 2%, or we're stopping:
        if( Math.abs(power - prevPower) > 0.02 || (power == 0.0 && prevPower != 0.0) ) {
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

    // Apply a rate + magnitude limit:
    double applyLimits(double desired, double last, double rateLimit, double maxMagnitude, double dt){
        // Calculate max allowed increase based on rate limit and passage of time:
        double maxIncrease = dt * rateLimit;
        double result;

        // Allow only that much increase, but instant decrease back to zero:
        if( last >= 0 ){
            result = Math.max(Math.min(desired, last + maxIncrease), 0 - maxIncrease);
        } else {
            result = Math.min(Math.max(desired, last - maxIncrease), 0 + maxIncrease);
        }
        return Math.max(Math.min(result, maxMagnitude), -maxMagnitude);
    }
    
    void drive(double forward, double right, double rotate)
    {
        double maxPower = 1;
        double maxSpeed = 1;  // make this slower for outreaches

        // This calculates the power needed for each wheel based on the amount of forward,
        // strafe right, and rotate
        double frontLeftPower = forward + right + rotate;
        double frontRightPower = forward - right - rotate;
        double backRightPower = forward + right - rotate;
        double backLeftPower = forward - right + rotate;


        // This is needed to make sure we don't pass > 1.0 to any wheel
        // It allows us to keep all of the motors in proportion to what they should
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

//    // Auto loop to smoothly drive forward/right a desired amount using odometry. Also resets the odometry so be careful.
//    void autoDriveNoRotation(double inchesFwd, double inchesRight, double speed)
//    {
//        // Reset the odometry, so robot is at position 0,0 and facing along X axis
//        localization.setPose(0, 0, 0);
//
//        double currentForward = 0, currentRight = 0;
//        double lastTime = runtime.seconds();
//        while( opModeIsActive()
//                && Math.abs(localization.getX()) < (inchesFwd - Calibration.AUTON_DIST_THRESHOLD)
//                && Math.abs(localization.getY()) < (inchesRight - Calibration.AUTON_DIST_THRESHOLD) ){
//            beginLoop("autoDriveForward");
//
//            double now = runtime.seconds();
//            double dt = now - lastTime;
//            lastTime = now;
//            if( inchesFwd != 0 ){
//                // Calculate forward command for smooth deceleration using kinematics equation:
//                double distFwd = inchesFwd - localization.getX();
//                double forward = Math.sqrt( 2 * Calibration.AUTON_ACCEL * distFwd );
//                // Use rate limiter to apply smooth acceleration:
//                currentForward = applyLimits(forward, currentForward, Calibration.AUTON_ACCEL, speed, dt);
//            }
//            if( inchesRight != 0 ){
//                // Calculate forward command for smooth deceleration using kinematics equation:
//                double distRight = inchesRight - localization.getY();
//                double right     = Math.sqrt( 2 * Calibration.AUTON_ACCEL * distRight );
//                // Use rate limiter to apply smooth acceleration:
//                currentRight = applyLimits(right, currentRight, Calibration.AUTON_ACCEL, speed, dt);
//            }
//            // Keep rotation on target:
//            double rotate = localization.getHeadingDeg() * Calibration.AUTON_ANGLE_GAIN_P;
//            drive(currentForward, currentRight, rotate);
//            telemetry.addData("Forward/Right/Rotate", "%.2f / %.2f / %.2f", currentForward, currentRight, rotate);
//        }
//        drive(0, 0, 0);
//    }

    /** Auto loop to smoothly drive to a desired X/Y position in a straight line using odometry.
     - Assumes zero speed at beginning and end of operation.
     - Locks heading to current value during operation.
     - Keeps trying forever until within 1 inch of target. */
    void autoSlideTo(double xInches, double yInches)
    {
        double originalHeading = Math.toDegrees( localization.getHeadingDeg() );
        autoSlideTo(xInches, yInches, originalHeading);
    }

    /** Auto loop to smoothly drive to a desired X/Y position in a straight line using odometry.
     - Assumes zero speed at beginning and end of operation.
     - Locks heading to given value during operation.
     - Keeps trying forever until within 1 inch of target. */
    void autoSlideTo(double xInches, double yInches, double headingDeg)
    {
        double currentSpeed = 0;
        double lastTime = runtime.seconds();
        double distanceToGo = localization.distanceTo(xInches, yInches);
        String label = String.format("autoSlideTo(%.1f, %.1f, %.1f)", xInches, yInches, headingDeg);

        while( opModeIsActive() && timesNotUp() && distanceToGo > Calibration.AUTON_DIST_THRESHOLD){
            beginLoop(label);

            // Calculate timestep:
            double now = runtime.seconds();
            double dt = now - lastTime;
            lastTime = now;

            // Calculate new distance and heading to target:
            distanceToGo = localization.distanceTo(xInches, yInches);
            double headingToGo = localization.directionTo(xInches, yInches);

            // Calculate max speed command for smooth _deceleration_ from this distance using kinematics equation:
            double speedToGo = Math.sqrt( 2 * Calibration.AUTON_ACCEL * distanceToGo );

            // Use rate limiter to apply smooth _acceleration_ and cap to desired travel speed
            currentSpeed = applyLimits(speedToGo, currentSpeed, Calibration.AUTON_ACCEL, Calibration.AUTON_SLIDE_SPEED, dt);

            // Turn speed and heading into forward/rotate:
            localization.convertSpeedDirection(speedToGo, headingToGo);

            // Keep rotation on target:
            double rotateP = localization.diffFromHeadingDeg(headingDeg) * Calibration.AUTON_ANGLE_GAIN_P;

            drive(localization.newForward, localization.newRight, rotateP);
            telemetry.addData("Forward/Right/Rotate", "%.2f / %.2f / %.2f",
                    localization.newForward, localization.newRight, rotateP);
        }
    }

    /** Auto sequence to drain a flower. Must be already lines up with robot center AUTON_DIST_FLOWER inches from the wall.
     *  Does not wait for flower bar to go down. Make sure you set it down before calling this!
     *  It does raise the flower bar afterwards though. */
    void autoDrainFlower()
    {
        double heading = localization.getHeadingSnappedTo(90);
        double startX = localization.getX();
        double startY = localization.getY();

        double drainX = startX + Math.cos(Math.toRadians(heading)) * Calibration.AUTON_DIST_FLOWER_DRAIN;
        double drainY = startY + Math.sin(Math.toRadians(heading)) * Calibration.AUTON_DIST_FLOWER_DRAIN;

        // Turn on intake. Flower bar should already be down, and we don't want to wait for it here.
        intakeOn();

        // jam into flower, wait 2 seconds
        autoSlideTo(drainX, drainY, heading);
        autoPause(2);

        // Back up to starting position, then do it again:
        autoSlideTo(startX, startY, heading);
        autoSlideTo(drainX, drainY, heading);
        autoPause(2);

        // back up to starting position, raise bar, and turn off intake
        autoSlideTo(startX, startY, heading);
        flowerBarUp();
        intakeOff();
    }

    /** Auto loop to just wait for a set number of seconds */
    void autoPause(double pauseSeconds){
        double end = runtime.seconds() + pauseSeconds;
        while( opModeIsActive() && runtime.seconds() < end ) {
            beginLoop("autoPause");
        }
    }

    /** Auto loop to smoothly rotate to a desired heading (CCW degrees field-relative).
     - Assumes zero rotation speed at beginning and end of operation.
     - Locks position current value during operation.
     - Keeps trying forever until within 1 degree of target for 0.25 sec. */
    void autoRotateTo(double newHeadingDeg)
    {
        double originalX = localization.getX();
        double originalY = localization.getY();

        double currentDegPerSec = 0;
        double now = runtime.seconds();
        double lastTime = now;
        double lastMisalignedTime = now;
        double degreesToGoCCW = -localization.diffFromHeadingDeg(newHeadingDeg);
        String label = String.format("autoRotateTo(%.1f, %.1f)", newHeadingDeg, Calibration.AUTON_ROTATE_SPEED);

        while( opModeIsActive() && timesNotUp() && now - lastMisalignedTime < Calibration.AUTON_ANGLE_SETTLING){
            beginLoop(label);

            // Calculate timestep:
            now = runtime.seconds();
            double dt = now - lastTime;
            lastTime = now;

            // Calculate new distance and heading to target:
            degreesToGoCCW = -localization.diffFromHeadingDeg(newHeadingDeg);

            // Determine whether we're in the settling window:
            if( Math.abs(degreesToGoCCW) > Calibration.AUTON_ANGLE_THRESH ){
                lastMisalignedTime = now;
            }

            // Calculate max speed command for smooth _deceleration_ from this distance using kinematics equation:
            double degPerSec = Math.sqrt( 2 * Calibration.AUTON_ANGLE_ACCEL * Math.abs(degreesToGoCCW) );
            if( Math.signum(degreesToGoCCW) < 0 ){
                degPerSec = -degPerSec;
            }

            // Use rate limiter to apply smooth _acceleration_ and cap to desired travel speed
            currentDegPerSec = applyLimits(degPerSec, currentDegPerSec, Calibration.AUTON_ANGLE_ACCEL, Calibration.AUTON_ROTATE_SPEED, dt);

            // Convert from deg/sec to motor power:
            double rotate = Localization.getRotateCommand(Math.toRadians(currentDegPerSec));

            // Apply control loop to keep position constant:
            double pX = (originalX - localization.getX()) * Calibration.AUTON_POSITION_GAIN_P;
            double pY = (originalY - localization.getY()) * Calibration.AUTON_POSITION_GAIN_P;
            localization.convertFieldRelative(pX, pY, rotate);

            drive(localization.newForward, localization.newRight, rotate);
            telemetry.addData("Forward/Right/Rotate", "%.2f / %.2f / %.2f",
                    localization.newForward, localization.newRight, rotate);
        }
    }

    /** Attempt to align to APRIL tag and fire the catapult. Returns true if fire happens before abort time. */
    public boolean autoAlignAndFire(double abortTime){
        boolean fired = false;
        while( opModeIsActive() && timesNotUp() && runtime.seconds() < abortTime ) {
            beginLoop("autoAlignAndFire");
            AlineToAprilTag();
            if( aprilSensing.OnTarget ){
                // Successfully aligned to april tag. Fire away!
                launchCatapult();
                fired = true;

                // Break out of loop
                break;
            }
        }
        // Stop motors:
        drive(0, 0, 0);

        setLEDTeamColor(); // Reset LED to team color after april alignment

        // Wait for catapult to finish launching (if it is)
        while( opModeIsActive() && waitingForLaunch() && runtime.seconds() < abortTime + 2 ) {
            beginLoop("autoAlignAndFire (Launching)");
        }

        return fired;
    }

    /** Auto loop to hold the desired distance from a rear and left wall using distance sensors.
     *  Stops at 30 seconds to prevent penalties. */
    public void autoHoldDistances(double inchesRear, double inchesLeft)
    {
        while( opModeIsActive() && timesNotUp() ){
            beginLoop("autoHoldDistances");
            holdPositionLeft(inchesRear, inchesLeft);
        }
    }


    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the OpMode running.
     */
    public void drivePosition(double speed, double timeoutS, double forward, double right, double rotate) {
        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newBackLeftTarget;
        int newBackRightTarget;

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // This calculates the power needed for each wheel based on the amount of forward,
            // strafe right, and rotate
            double frontLeftInches = forward + right + rotate;
            double frontRightInches = forward - right - rotate;
            double backRightInches = forward + right - rotate;
            double backLeftInches = forward - right + rotate;

            // Determine new target position, and pass to motor controller
            newFrontLeftTarget = frontLeftDrive.getCurrentPosition() + (int)(frontLeftInches * Calibration.COUNTS_PER_INCH);
            newBackLeftTarget = backLeftDrive.getCurrentPosition() + (int)(backLeftInches * Calibration.COUNTS_PER_INCH);
            newFrontRightTarget = frontRightDrive.getCurrentPosition() + (int)(frontRightInches * Calibration.COUNTS_PER_INCH);
            newBackRightTarget = backRightDrive.getCurrentPosition() + (int)(backRightInches * Calibration.COUNTS_PER_INCH);
            frontLeftDrive.setTargetPosition(newFrontLeftTarget);
            frontRightDrive.setTargetPosition(newFrontRightTarget);
            backLeftDrive.setTargetPosition(newBackLeftTarget);
            backRightDrive.setTargetPosition(newBackRightTarget);

            // Turn On RUN_TO_POSITION
            frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            double start = runtime.seconds();
            frontLeftDrive.setPower(Math.abs(speed));
            frontRightDrive.setPower(Math.abs(speed));
            backLeftDrive.setPower(Math.abs(speed));
            backRightDrive.setPower(Math.abs(speed));


            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                   (runtime.seconds()-start < timeoutS) &&
                   (frontLeftDrive.isBusy() || frontRightDrive.isBusy() ||
                     backLeftDrive.isBusy() || backRightDrive.isBusy()) ) {
                beginLoop("drivePosition");

                // Display it for the driver.
                telemetry.addData("Running to",  " %7d :%7d", newFrontLeftTarget, newFrontRightTarget);
                telemetry.addData("Currently at",  " at %7d :%7d",
                                            frontLeftDrive.getCurrentPosition(), frontRightDrive.getCurrentPosition());
                telemetry.update();
            }
            // Allow settling time for encoder position
            sleep(250);

            // Stop all motion;
            frontLeftDrive.setPower(0);
            frontRightDrive.setPower(0);
            backLeftDrive.setPower(0);
            backRightDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //sleep(250);   // optional pause after each move.
        }
    }
}
