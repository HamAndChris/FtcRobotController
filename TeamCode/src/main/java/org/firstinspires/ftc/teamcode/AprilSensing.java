package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

import org.firstinspires.ftc.teamcode.Calibration;

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

        // Initialize camera:
        //aprilTag = AprilTagProcessor.easyCreateWithDefaults();

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
            // ... these parameters are fx, fy, cx, cy.

            .build();

        VisionPortal.Builder builder = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new android.util.Size(1280, 800)) // match supported resolution
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .addProcessor(aprilTag);
        visionPortal = builder.build();
    }

    // Switch teams:
    public void setTeamColor(String teamColor){
        TeamColor = teamColor;
    }
    
    public void performSensing(double forward, double right, double rotate) {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());
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
                telemetry.addData("\n==== Tag Cluster", name);
                telemetry.addData("Percent tags found", "%d", clusterDet.percentClusterFound);
                telemetry.addData("XYZ", "%6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z);
                telemetry.addData("PRY","%6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw);
                telemetry.addData("RBE","%6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation);

                // Find the hive that's the correct color, and tipped upwards (not upside down)
                if( name.contains(TeamColor) && Math.abs(detection.ftcPose.roll) < 90 ){
                    Rotate = (Calibration.APRIL_TARGET_YAW - detection.ftcPose.bearing) // was previously using "yaw". Verify this still works.
                            * Calibration.APRIL_YAW_GAIN_P;
                    Forward = (Calibration.APRIL_TARGET_DISTANCE - detection.ftcPose.range)
                            * -Calibration.APRIL_DIST_GAIN_P;
                    telemetry.addData("Rotate/Forward", "%3.2f / %3.2f", Rotate, Forward);

                    // Allow strafing, with rotation added:
                    if( ALLOW_STRAFING_IN_ALIGNMENT ) {
                        double rotationRadius = 30; // inches
                        //                    double rightSpeed = right * Calibration.MAX_SPEED_INCHES_PER_S / 5; // fudge factor
                        //                    double extraRotate = -Math.asin(rightSpeed/rotationRadius);
                        //                    Rotate += extraRotate;

                        // Use actual lateral velocity to calculate rotation rate needed:
                        double rightSpeed = -localization.getRobotVy();
                        double extraRadsPerSec = -Math.asin(rightSpeed / rotationRadius);
                        Rotate += localization.getRotateCommand(extraRadsPerSec);
                    }
                    else {
                        Right = 0;
                    }

                    Detected = true;

                    // Apply threshold to determine if we are on target enough to fire:
                    if( Math.abs(Rotate) < 0.05 && Math.abs(Forward) < 0.05 ){
                        OnTarget = true;
                    }
                }
            }
        }   // end for() loop

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");

        telemetry.update();
    }   // end method telemetryAprilTag()

}
