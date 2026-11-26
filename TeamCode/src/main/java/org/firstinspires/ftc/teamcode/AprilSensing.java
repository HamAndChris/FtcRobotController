package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

import org.firstinspires.ftc.robotcore.external.navigation.*;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class AprilSensing {
    private String TeamColor;
    private final AprilTagProcessor aprilTag;
    private final VisionPortal visionPortal;
    private final Telemetry telemetry;
    private final Localization localization;

    public double Forward = 0, Rotate = 0, Right = 0; // Outputs after calling AlineToAprilTag()
    public boolean Detected = false, OnTarget = false; // Whether we found a tag and are in an acceptable firing position

    public static final boolean ALLOW_STRAFING_IN_ALIGNMENT = true;

    public AprilSensing(String teamColor, HardwareMap hardwareMap, Telemetry telem, Localization localization){
        TeamColor = teamColor;
        telemetry = telem;
        this.localization = localization;

        // Set camera orientation so we can use robotPose for localization.
        // See: https://ftc-docs.firstinspires.org/en/latest/apriltag/vision_portal/apriltag_localization/apriltag-localization.html

        // Measured 11/18/2026

        // Origin location: Center of the robot at field height
        // Axes orientation: +x right, +y forward, +z upward
        Position cameraPosition = new Position(DistanceUnit.INCH, Calibration.APRIL_CAMERA_POS_X, Calibration.APRIL_CAMERA_POS_Y, Calibration.APRIL_CAMERA_HEIGHT, 0);

        // If all values are zero (no rotation), that implies the camera is pointing straight up.
        // In most cases, you’ll need to set the pitch to -90 degrees (rotation about the x-axis),
        // meaning the camera is horizontal. Use a yaw of 0 if the camera is pointing forwards,
        // +90 degrees if it’s pointing straight left, -90 degrees for straight right, etc.
        // You can also set the roll to +/-90 degrees if it’s vertical, or 180 degrees if it’s upside-down.
        YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
                0, - Calibration.APRIL_CAMERA_INCLINE, 0, 0);

        // Initialize camera:
        //aprilTag = AprilTagProcessor.easyCreateWithDefaults();


        // Allow instantiation with no hardware for unit testing
        if( hardwareMap == null ) {
            aprilTag = null;
            visionPortal = null;
        } else {
            // Create the AprilTag processor.
            aprilTag = new AprilTagProcessor.Builder()
                    // The following default settings are available to un-comment and edit as needed.
                    //.setDrawAxes(true) // Changed in V12.0
                    //.setDrawTagOutline(true)
                    //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                    //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                    //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                    //.setDrawCubeProjection(false)

                    // == CAMERA CALIBRATION ==
                    // If you do not manually specify calibration parameters, the SDK will attempt
                    // to load a predefined calibration for your camera.
                    .setLensIntrinsics(908.758f, 908.758f, 696.345f, 376.979f) // Should be able to comment this out if the XML file worked
                    // ... these parameters are fx, fy, cx, cy

                    // Set the camera location so we can use robotPose:
                    .setCameraPose(cameraPosition, cameraOrientation)

                    // Apply the custom field locations and orientations we determined with LINEAR ALGEBRA:
                    .setTagLibrary(CustomAprilTagGameDatabase.getCurrentGameTagLibrary())

                    .build();

            VisionPortal.Builder builder = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .setCameraResolution(new android.util.Size(1280, 800)) // match supported resolution
                    .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                    .addProcessor(aprilTag);
            visionPortal = builder.build();
        }
    }

    // Switch teams:
    public void setTeamColor(String teamColor){
        TeamColor = teamColor;
    }

    /** Read the April tags visible in the camera, and if there is a tipped-up hive in the correct color,
     *  calculate the proper Forward/Right/Rotate commands needed to align to it.
     * @param forward       Gamepad command to pass-through if no detection is found
     * @param right         Gamepad command to pass-through if no detection is found (or to strafe while aligned)
     * @param rotate        Gamepad command to pass-through if no detection is found
     */
    public void performSensing(double forward, double right, double rotate){
        performSensing(forward, right, rotate, false);
    }

    /** Read the April tags visible in the camera, and if there is a tipped-up hive in the correct color,
     *  calculate the proper Forward/Right/Rotate commands needed to align to it.
     * @param forward       Gamepad command to pass-through if no detection is found
     * @param right         Gamepad command to pass-through if no detection is found (or superAlign is false)
     * @param rotate        Gamepad command to pass-through if no detection is found
     * @param superAlign    If true, will strafe to right in front of the hive. False will just align angle/distance.
     */
    public void performSensing(double forward, double right, double rotate, boolean superAlign) {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        //telemetry.addData("# AprilTags Detected", currentDetections.size());
        Detected = false;
        OnTarget = false;
        Rotate = rotate;
        Forward = forward;
        Right = right;

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection instanceof AprilTagClusterDetection) {
                AprilTagClusterDetection clusterDet = (AprilTagClusterDetection) detection;
                String name = clusterDet.metadata.name;
                telemetry.addData("\n==== Tag Cluster", "%s, (%d %%)", name, clusterDet.percentClusterFound);
                telemetry.addData("XYZ", "%6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z);
                telemetry.addData("PRY","%6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw);
                telemetry.addData("RBE","%6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation);
                Position pos = detection.robotPose.getPosition();
                YawPitchRollAngles orx = detection.robotPose.getOrientation();
                telemetry.addData("Robot","XYZ %6.1f %6.1f %6.1f RPY %6.1f %6.1f %6.1f",
                        pos.x, pos.y, pos.z, orx.getRoll(), orx.getPitch(), orx.getYaw() + 90); // Reported reference frame is rotated 90 for some reason
                // Debugging: also print hive field position
                //telemetry.addData("Hive Pos","XYZ %s", clusterDet.metadata.fieldPosition);
                //telemetry.addData("Hive Quat","%s", clusterDet.metadata.fieldOrientation);
                telemetry.addData("Tag Height", "%.1f", detectionHeight(clusterDet));

                // Find the hive that's the correct color, and tipped upwards (not upside down)
                if( name.contains(TeamColor) && hiveIsTippedUp(clusterDet) ){
                    Detected = true;

                    // Apply the control loop to it! (this is a separate function now so it can be unit tested)
                    applyAlignmentControlLoop(clusterDet, right, superAlign);

                    // Apply threshold to determine if we are on target enough to fire:
                    if( Math.abs(Rotate) < 0.05 && Math.abs(Forward) < 0.05 ){
                        OnTarget = true;

                        // If the robot pose is also a good reading, go ahead and update localization!
                        if( robotPoseValid(clusterDet) ){
                            localization.setPose(pos.x, pos.y, orx.getYaw() + 90); // Reported reference frame is rotated 90 for some reason
                            telemetry.addLine("Location Updated!");
                        }
                    }
                }
            }
        }   // end for() loop

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");
    }   // end method telemetryAprilTag()

    /** Apply alignment control loops to the detected tag cluster */
    void applyAlignmentControlLoop(AprilTagClusterDetection clusterDet, double right, boolean superAlign)
    {
        // Rotation control loop:
        Rotate = (Calibration.APRIL_TARGET_BEARING - clusterDet.ftcPose.bearing)
                * Calibration.APRIL_BEARING_GAIN_P;

        // Radial speed control loop (previously Forward only):
        double radial = (Calibration.APRIL_TARGET_DISTANCE - clusterDet.ftcPose.range)
                * -Calibration.APRIL_DIST_GAIN_P;

        // Tangential control (either feedback loop, user input, or nothing):
        double tangential;
        if( superAlign ){
            // Tangential speed control loop:
            tangential = (Calibration.APRIL_TARGET_YAW - (clusterDet.ftcPose.yaw - clusterDet.ftcPose.bearing))
                    * Calibration.APRIL_YAW_GAIN_P;
        }
        else if( ALLOW_STRAFING_IN_ALIGNMENT  ) {
            // Allow strafing, with rotation added:
            tangential = right;
        }
        else {
            // No strafing allowed.
            tangential = 0;
        }

        // Mix tangential and radial control loops into forward/right commands:
        double bearing = trueBearing(clusterDet);
        Forward = radial * Math.cos(bearing) - tangential * Math.sin(bearing);
        Right = radial * Math.sin(bearing) - tangential * Math.cos(bearing);

        // Use actual lateral velocity to calculate extra rotation rate needed to maintain target:
        double extraRadsPerSec = angularRateAboutTag(clusterDet);
        Rotate += Localization.findRotateCommand(extraRadsPerSec);

        telemetry.addData("Forward/Right/Rotate", "%3.2f / %3.2f / %3.2f", Forward, Right, Rotate);
    }

    /** Determine whether it is safe to use this April tag cluster to update field position */
    boolean robotPoseValid(AprilTagClusterDetection cluster){
        // Need to make sure this hive is a good detection, the right color, tipped up, and not bogus
        Position pos = cluster.robotPose.getPosition();
        YawPitchRollAngles orx = cluster.robotPose.getOrientation();
        return cluster.percentClusterFound > 74   // At least 3 APRIL tags are detected
                && Math.abs(pos.z) < 6            // Valid robot pose (near the floor)
                && Math.abs(orx.getPitch()) < 5   // Valid robot pose (robot upright)
                && Math.abs(orx.getRoll()) < 5;   // Valid robot pose (robot upright)
    }

    /** Calculate the robot's translational speed orthogonal to the radius to the tag (inch/sec, CCW = positive) */
    double tangentialSpeedAboutTag(AprilTagClusterDetection det){
        // Use built-in pose estimation which includes the camera position on robot
        double xDiff = det.metadata.fieldPosition.get(0) - det.robotPose.getPosition().x;
        double yDiff = det.metadata.fieldPosition.get(1) - det.robotPose.getPosition().y;
        double xyBearing = Math.atan2(yDiff, xDiff);
        double heading = det.robotPose.getOrientation().getYaw(AngleUnit.RADIANS) + Math.PI/2;
        double headingDiff = xyBearing - heading;

        double xVel = localization.getRobotVx();
        double yVel = localization.getRobotVy();
        return Math.sin(headingDiff) * xVel - Math.cos(headingDiff) * yVel;
    }

    /** Calculate angular rate of rotation around the tag cluster (in radians/sec CCW) */
    double angularRateAboutTag(AprilTagClusterDetection det){
        double dist = detectionXYDistance(det);
        return (dist > 0.01) ? (tangentialSpeedAboutTag(det) / dist) : 0;
    }

    /** Calculate the height of the detection based on trigonometry */
    double detectionHeight(AprilTagDetection det){
        double elevation = Math.toRadians(det.ftcPose.elevation + 90 - Calibration.APRIL_CAMERA_INCLINE);
        return Math.sin(elevation) * det.ftcPose.range + Calibration.APRIL_CAMERA_HEIGHT;
    }

    /** Calculate the height of the detection based on trigonometry */
    double detectionXYDistance(AprilTagClusterDetection det){
        // Use built-in pose estimation which includes the camera position on robot
        double xDiff = det.metadata.fieldPosition.get(0) - det.robotPose.getPosition().x;
        double yDiff = det.metadata.fieldPosition.get(1) - det.robotPose.getPosition().y;
        return Math.hypot(xDiff, yDiff);
//        double elevation = Math.toRadians(det.ftcPose.elevation + 90 - Calibration.APRIL_CAMERA_INCLINE);
//        double bearingRads = Math.toRadians(det.ftcPose.bearing);
//        return Math.cos(elevation) * det.ftcPose.range + Calibration.APRIL_CAMERA_POS_Y;
    }

    /** Calculate the "true bearing" from robot to the detection. Includes camera position offset */
    double trueBearing(AprilTagClusterDetection det){
        double xDiff = det.metadata.fieldPosition.get(0) - det.robotPose.getPosition().x;
        double yDiff = det.metadata.fieldPosition.get(1) - det.robotPose.getPosition().y;
        double xyBearing = Math.atan2(yDiff, xDiff);
        double heading = det.robotPose.getOrientation().getYaw(AngleUnit.RADIANS) + Math.PI/2;
        return AngleUnit.normalizeRadians(xyBearing - heading);
    }

    /** Determine whether the detected hive is in the "up" position */
    boolean hiveIsTippedUp(AprilTagDetection det){
        return Math.abs(det.ftcPose.roll) < 90  // Not reading hive from wrong side
//                && det.ftcPose.pitch < 20;      // Tipped sufficiently upward // TODO: Find a better way to calculate this
            && detectionHeight(det) >= Calibration.APRIL_HIVE_MIN_HEIGHT;
    }
}
