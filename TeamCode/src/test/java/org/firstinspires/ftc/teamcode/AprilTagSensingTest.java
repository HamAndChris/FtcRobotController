package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterMetadata;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseRaw;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import static org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase.getIntoTheDeepTagLibrary;
import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.CALLS_REAL_METHODS;
import static org.mockito.Mockito.mock;

/** Class to test our April tag sensing and pose calculation logic. */
public class AprilTagSensingTest {
    private Localization localization;
    private AprilSensing aprilSensing;

    @BeforeEach
    void setUp(){

        // Create a mock that executes real methods, but skips
        // the constructor (which requires robot hardware).
        localization = mock(Localization.class, CALLS_REAL_METHODS);
        aprilSensing = new AprilSensing("RED", null, null, localization);
    }

    /** Make sure our quaternion generators are working and we understand how they are used */
    @Test public void testQuaternionRotation(){
        // Starting orientation of the April tags is face down, with the top edge facing an observer to the South (negative Y).
        // Thus, flipping it "up" requires first rotating -90 along the X axis.
        // The intoTheDeep Blue Alliance tag should match this orientation.
        Quaternion xNegQuat = CustomAprilTagGameDatabase.xRotationQuat(-90);
        compareQuats("-X rotation should match BlueAllianceWall",
                xNegQuat,
                getIntoTheDeepTagLibrary().lookupTag(12).fieldOrientation);

        // Rotating this "flipped up" tag to the left wall then requires rotating 90 degrees in Z.
        // When multiplying quaternions, the left-hand one is applied in the *global reference frame*.
        // The IntoTheDeep Red Audience Wall tag should match this orientation.
        Quaternion zQuat = CustomAprilTagGameDatabase.zRotationQuat(90);
        compareQuats("Zrot * -Xrot should match RedAudienceWall",
                zQuat.multiply(xNegQuat, 0),
                getIntoTheDeepTagLibrary().lookupTag(16).fieldOrientation);

        // Another way to get the red audience wall would be rotating it along Z first, then negative Y.
        Quaternion yNegQuat = CustomAprilTagGameDatabase.yRotationQuat(-90);
        compareQuats("-Yrot * Zrot should match RedAudienceWall",
                yNegQuat.multiply(zQuat, 0),
                getIntoTheDeepTagLibrary().lookupTag(16).fieldOrientation);

        // Sanity check for the red rear wall - should be negative Z first, then positive Y
        Quaternion yQuat = CustomAprilTagGameDatabase.yRotationQuat(90);
        Quaternion zNegQuat = CustomAprilTagGameDatabase.zRotationQuat(-90);
        compareQuats("Yrot * -Zrot should match RedRearWall",
                yQuat.multiply(zNegQuat, 0),
                getIntoTheDeepTagLibrary().lookupTag(14).fieldOrientation);
    }

    @Test public void testTrigonometry(){
        AprilTagLibrary library = CustomAprilTagGameDatabase.getBioBuzzTagLibrary();
        AprilTagClusterMetadata redScoring = library.lookupCluster(30);
        AprilTagPoseFtc ftcPose = new AprilTagPoseFtc(0, 0, 0, 0, 0, 0, 0, 0, 0);
        AprilTagPoseRaw rawPose = new AprilTagPoseRaw(0, 0, 0, null);
        double tagY = CustomAprilTagGameDatabase.CLUSTER_Y;
        double tagX = CustomAprilTagGameDatabase.FIELD_CENTER_X - CustomAprilTagGameDatabase.CLUSTER_X_OFFSET;
        double head = Math.toDegrees(Math.atan2(tagY, tagX));

        double[][] testCases = {
                // ID, RobotX, RobotY, RobotHeading, VelFwd, VelLeft, TrueBearing, TangentialSpeed
                { 30,  0,           0,            0,      0,       0,        head,        0}, // Red scoring, from origin
                { 30,  0,           0,         head,      0,       0,           0,        0}, // Red scoring, from origin, pointed at hive
                { 30,  0,        tagY,            0,     10,       0,           0,        0}, // Red scoring, driving straight at hive
                { 30,  0,        tagY,            0,      0,      10,           0,      -10}, // Red scoring, strafing
                { 30,  tagX-20,  tagY,           45,     10,       0,         -45,  -7.0711}, // Red scoring, forward at 45 degrees (CW)
                { 30,  tagX-20,  tagY,          -45,     10,       0,          45,   7.0711}, // Red scoring, forward at -45 degrees (CCW)
                { 30,  tagX-20,  tagY,          135,     10,       0,        -135,  -7.0711}, // Red scoring, forward at 135 degrees (CW)
                { 30,  tagX-20,  tagY,          135,      0,     -10,        -135,  -7.0711}  // Red scoring, strafing at 135 degrees (CCW)
        };
        for (double[] tc : testCases) {
            AprilTagClusterMetadata metadata = library.lookupCluster((int)tc[0]);
            double robotPoseX = tc[1];
            double robotPoseY = tc[2];
            double robotPoseHeading = tc[3];
            double velX = tc[4];
            double velY = tc[5];
            double bearing = tc[6];
            double angularSpeed = tc[7];
            Pose3D robotPose = new Pose3D(
                    new Position(DistanceUnit.INCH, robotPoseX, robotPoseY, 0, 0),
                    new YawPitchRollAngles(AngleUnit.DEGREES, robotPoseHeading - 90, 0, 0, 0));
            AprilTagClusterDetection det = new AprilTagClusterDetection(100, metadata, DistanceUnit.INCH, ftcPose, rawPose, robotPose, 0);
            localization.overrideRobotVelocity(velX, velY);

            // Test bearing calculation:
            double result = Math.toDegrees(aprilSensing.trueBearing(det));
            String msg = String.format("TrueBearing: Robot X=%.0f, Y=%.0f Head=%.2f",
                    robotPoseX, robotPoseY, robotPoseHeading);
            assertEquals(bearing, result, 1e-2, msg);

            // Test angular speed calculation:
            result = aprilSensing.tangentialSpeedAboutTag(det);
            msg = String.format("TangentialSpeedAboutTag: Robot X=%.0f, Y=%.0f Head=%.0f, VelX=%.0f, VelY=%.0f (Bearing = %.0f)",
                    robotPoseX, robotPoseY, robotPoseHeading, velX, velY, bearing);
            assertEquals(angularSpeed, result, 1e-3, msg);
        }
    }

    /** Test the control loop logic so everything is at least the right direction */
    @Test public void testControlLoop(){
        AprilTagLibrary library = CustomAprilTagGameDatabase.getBioBuzzTagLibrary();
        AprilTagClusterMetadata redScoring = library.lookupCluster(30);
        AprilTagPoseFtc ftcPose = new AprilTagPoseFtc(0, 0, 0, 0, 0, 0, 0, 0, 0);
        AprilTagPoseRaw rawPose = new AprilTagPoseRaw(0, 0, 0, null);
        double tagY = CustomAprilTagGameDatabase.CLUSTER_Y;
        double tagX = CustomAprilTagGameDatabase.FIELD_CENTER_X - CustomAprilTagGameDatabase.CLUSTER_X_OFFSET;
        double head = Math.toDegrees(Math.atan2(tagY, tagX));
        double range = 60; // Rather than calculating what range would really be for each point

        double[][] testCases = {
                // ID, RobotX, RobotY, RobotHeading, VelFwd, VelLeft, Forward, Right, Rotate
                { 30,       0,      0,            0,      0,       0,        0,        0}, // Red scoring, from origin
                { 30, tagX-20,   tagY,          135,      0,     -10,        -135,  -7.0711}  // Red scoring, strafing at 135 degrees (CCW)
        };
        for (double[] tc : testCases) {
        }

    }

    /** Test the hive tipped up checking logic */
    @Test public void testTippedUp(){

        double[][] testCases = {
                //   P,    R,    Y,    R,    B,    E,  tip
                { 26.8,  1.8, -2.9, 53.9,  0.0, -2.9,    0},
                {  8.7, -2.1, -4.2, 54.1,  4.0,  6.4,    1},
                { 10.9, -1.4, -3.0, 53.9, -2.9,  5.7,    1}
        };
        for (double[] tc : testCases) {
            AprilTagPoseFtc ftcPose = new AprilTagPoseFtc(0, 0, 0,
                    tc[2], tc[0], tc[1], tc[3], tc[4], tc[5]);
            AprilTagDetection det = new AprilTagClusterDetection(100, null, DistanceUnit.INCH, ftcPose, null, null, 0);
            double height = aprilSensing.detectionHeight(det);
            String msg = String.format("PRY: %.1f %.1f %.1f, RBE: %.1f, %.1f, %.1f (height %.1f)", tc[0], tc[1], tc[2], tc[3], tc[4], tc[5], height);
            if( tc[6] > 0.5 ){
                assertTrue(aprilSensing.hiveIsTippedUp(det), msg);
            } else {
                assertFalse(aprilSensing.hiveIsTippedUp(det), msg);
            }
        }


    }

    public static void compareQuats(String testName, Quaternion result, Quaternion desired){
        assertAll( testName,
                () -> assertEquals(result.w, desired.w, 1e-6, "W should match"),
                () -> assertEquals(result.x, desired.x, 1e-6, "X should match"),
                () -> assertEquals(result.y, desired.y, 1e-6, "Y should match"),
                () -> assertEquals(result.z, desired.z, 1e-6, "Z should match")
        );
    }
}
