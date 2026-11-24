package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.junit.jupiter.api.Test;

import static org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase.getIntoTheDeepTagLibrary;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertAll;

/** Class to test our April tag sensing and pose calculation logic. */
public class AprilTagSensingTest {
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

    public static void compareQuats(String testName, Quaternion result, Quaternion desired){
        assertAll( testName,
                () -> assertEquals(result.w, desired.w, 1e-6, "W should match"),
                () -> assertEquals(result.x, desired.x, 1e-6, "X should match"),
                () -> assertEquals(result.y, desired.y, 1e-6, "Y should match"),
                () -> assertEquals(result.z, desired.z, 1e-6, "Z should match")
        );
    }
}
