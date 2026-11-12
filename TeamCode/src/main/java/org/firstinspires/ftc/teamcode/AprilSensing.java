package org.firstinspires.ftc.teamcode;

import java.util.List;

import org.firstinspires.ftc.teamcode.Calibration;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class AprilSensing {
    private final String TeamColor;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private Telemetry telemetry;

    public double Forward = 0, Rotate = 0, Right = 0; // Outputs after calling AlineToAprilTag()
    public boolean Detected = false, OnTarget = false; // Whether we found a tag and are in an acceptable firing position

    public AprilSensing(String teamColor, HardwareMap hardwareMap, Telemetry telem){
        TeamColor = teamColor;
        telemetry = telem;

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
    
    public void performSensing() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());
        Detected = false;
        OnTarget = false;
        Rotate = 0;
        Forward = 0;
        Right = 0;

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection instanceof AprilTagClusterDetection) {
                AprilTagClusterDetection clusterDet = (AprilTagClusterDetection) detection;
                String name = clusterDet.metadata.name;
                telemetry.addLine(String.format("\n==== Tag Cluster (%s)", name));
                telemetry.addLine(String.format("Percent tags found: %d", clusterDet.percentClusterFound));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));

                // Find the hive that's the correct color, and tipped upwards (not upside down)
                if( name.contains(TeamColor) && Math.abs(detection.ftcPose.roll) < 90 ){
                    Rotate = (Calibration.APRIL_TARGET_YAW - detection.ftcPose.bearing) // was previously using "yaw". Verify this still works.
                            * Calibration.APRIL_YAW_GAIN_P;
                    Forward = (Calibration.APRIL_TARGET_DISTANCE - detection.ftcPose.range)
                            * -Calibration.APRIL_DIST_GAIN_P;
                    Right = 0;
                    telemetry.addLine(String.format("rotate: %3.2f, forward: %3.2f", rotate, forward));
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
