package org.firstinspires.ftc.teamcode;
/*
 * Copyright (c) 2023 FIRST
 *
 * All rights reserved.
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
 * Neither the name of FIRST nor the names of its contributors may be used to
 * endorse or promote products derived from this software without specific prior
 * written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
 * TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterMemberMetadata;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterMetadata;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;

import java.util.ArrayList;

public class CustomAprilTagGameDatabase
{
    // Measurements pulled from official OnShape field CAD
    public static final double FIELD_CENTER_Y   = 72;
    public static final double FIELD_CENTER_X   = 72;
    public static final double TAG_HEIGHT       = 49.654; // Height of tag cluster center from floor
    public static final double CLUSTER_Y_OFFSET = 25.5/2; // Y distance from cluster center to driver wall
    public static final double CLUSTER_X_OFFSET = 12.876; // X distance from cluster center to field center
    public static final double TAG_INCLINE      = 28.5;     // Degrees above horizontal

    /**
     * Get the {@link AprilTagLibrary} for the current season game, plus sample tags
     * @return the {@link AprilTagLibrary} for the current season game, plus sample tags
     */
    public static AprilTagLibrary getCurrentGameTagLibrary()
    {
        return new AprilTagLibrary.Builder()
                .addLibrary(getSampleTagLibrary())
                .addLibrary(getBioBuzzTagLibrary())
                .build();
    }

    // Helper functions to generate the quaternions we need. Based on understanding from this wonderful article:
    // https://medium.com/@dmdinithipurna/understanding-quaternions-67cf263c276f

    /** Generate a quaternion that applies rotation in the X axis by the specified degrees. */
    public static Quaternion xRotationQuat(double degrees){
        float w = (float) Math.cos(Math.toRadians(degrees/2));
        float x = (float) Math.sin(Math.toRadians(degrees/2));
        float y = 0;
        float z = 0;
        return new Quaternion(w, x, y, z, 0);
    }
    /** Generate a quaternion that applies rotation in the X axis by the specified degrees. */
    public static Quaternion yRotationQuat(double degrees){
        float w = (float) Math.cos(Math.toRadians(degrees/2));
        float x = 0;
        float y = (float) Math.sin(Math.toRadians(degrees/2));
        float z = 0;
        return new Quaternion(w, x, y, z, 0);
    }
    /** Generate a quaternion that applies rotation in the X axis by the specified degrees. */
    public static Quaternion zRotationQuat(double degrees){
        float w = (float) Math.cos(Math.toRadians(degrees/2));
        float x = 0;
        float y = 0;
        float z = (float) Math.sin(Math.toRadians(degrees/2));
        return new Quaternion(w, x, y, z, 0);
    }

    /**
     * Get the {@link AprilTagLibrary} for the BioBuzz FTC game, modified to include field position as we (team 8487)
     * intend to use them. That is, we will only localize off hives that are OUR OWN ALLIANCE, and TIPPED UP.
     * Also, our field orientation treats the left wall of our alliance's side as the origin.
     * Therefore, every tag will be oriented 30 degrees up from horizontal, and red audience / blue scoring will be identical.
     * Author: Chris Hammond, #8487 RoboToasters mentor, 2026
     *
     * @return the {@link AprilTagLibrary} for the BioBuzz FTC game
     */
    public static AprilTagLibrary getBioBuzzTagLibrary()
    {
        // Starting orientation of the April tags is face down, with the top edge facing an observer to the South (negative Y).
        // When multiplying quaternions, the left-hand operand is applied in the *global reference frame*.
        // Thus, to get the hive tag orientation, we must first rotate the tag in Z so the top edge faces the desired wall,
        // then tip that side up by 30 degrees.
        Quaternion rightOrientation = yRotationQuat(-TAG_INCLINE).multiply( zRotationQuat( 90), 0 );
        Quaternion leftOrientation  = yRotationQuat( TAG_INCLINE).multiply( zRotationQuat(-90), 0 );

        // XYZ coordinates are based on alliance side, with origin at left end of driver's wall.
        float rightSideX = (float) (FIELD_CENTER_X + CLUSTER_X_OFFSET);
        float leftSideX  = (float) (FIELD_CENTER_X - CLUSTER_X_OFFSET);
        float y = (float) (FIELD_CENTER_Y - CLUSTER_Y_OFFSET);
        float z = (float) TAG_HEIGHT;
        VectorF leftLocation  = new VectorF(leftSideX,  y, z);
        VectorF rightLocation = new VectorF(rightSideX, y, z);

        // Red: scoring is on left
        AprilTagClusterMetadata redScoring = new AprilTagClusterMetadata(
                getBioBuzzCluster(30),
                "RED SCORING",
                "RED SCORING",
                leftLocation,
                DistanceUnit.INCH,
                leftOrientation);

        // Red: audience is on right
        AprilTagClusterMetadata redAudience = new AprilTagClusterMetadata(
                getBioBuzzCluster(34),
                "RED AUDIENCE",
                "RED AUDIENCE",
                rightLocation,
                DistanceUnit.INCH,
                rightOrientation);

        // Blue: audience is on left
        AprilTagClusterMetadata blueAudience = new AprilTagClusterMetadata(
                getBioBuzzCluster(38),
                "BLUE AUDIENCE",
                "BLUE AUDIENCE",
                leftLocation,
                DistanceUnit.INCH,
                leftOrientation);

        // Blue: scoring is on the left
        AprilTagClusterMetadata blueScoring = new AprilTagClusterMetadata(
                getBioBuzzCluster(42),
                "BLUE SCORING",
                "BLUE SCORING",
                rightLocation,
                DistanceUnit.INCH,
                rightOrientation);

        return new AprilTagLibrary.Builder()
                .addCluster(redScoring)
                .addCluster(redAudience)
                .addCluster(blueAudience)
                .addCluster(blueScoring)
                .build();
    }

    private static ArrayList<AprilTagClusterMemberMetadata> getBioBuzzCluster(int firstID) {

        final double tagSize = 3.25;
        final float  offsetY = 7.1874f;
        final float  offsetZ = -5.622f;

        final VectorF TagL1 = new VectorF(-6.50f, offsetY, offsetZ);
        final VectorF TagL2 = new VectorF(-2.75f, offsetY, offsetZ);
        final VectorF TagR1 = new VectorF( 2.75f, offsetY, offsetZ);
        final VectorF TagR2 = new VectorF( 6.50f, offsetY, offsetZ);

        ArrayList<AprilTagClusterMemberMetadata> cluster = new ArrayList<>();

        cluster.add(new AprilTagClusterMemberMetadata(firstID, TagL1, tagSize));
        cluster.add(new AprilTagClusterMemberMetadata(firstID + 1, TagL2, tagSize));
        cluster.add(new AprilTagClusterMemberMetadata(firstID + 2, TagR1, tagSize));
        cluster.add(new AprilTagClusterMemberMetadata(firstID + 3, TagR2, tagSize));

        return cluster;
    }

    /**
     * Get the {@link AprilTagLibrary} for the tags used in the sample OpModes
     * @return the {@link AprilTagLibrary} for the tags used in the sample OpModes
     * Library contains 4 single AprilTags, and one cluster of 2 AprilTags.
     */
    public static AprilTagLibrary getSampleTagLibrary()
    {
        ArrayList<AprilTagClusterMemberMetadata> clstMem = new ArrayList<>();

        clstMem.add(new AprilTagClusterMemberMetadata(
                581, new VectorF(-2.0f, 0.0f, 0.0f), 3.0));
        clstMem.add(new AprilTagClusterMemberMetadata(
                582, new VectorF(2.0f,  0.0f, 0.0f), 3.0));

        AprilTagClusterMetadata clusterMetadata = new AprilTagClusterMetadata(
                clstMem,
                "Center Goal",
                "GOAL",
                new VectorF(0,0,0),
                DistanceUnit.INCH,
                Quaternion.identityQuaternion());

        return new AprilTagLibrary.Builder()
                .addTag(583, "Robbie",
                        2, DistanceUnit.INCH)
                .addTag(584, "Number 5",
                        4, DistanceUnit.INCH)
                .addTag(585, "C3PO",
                        6, DistanceUnit.INCH)
                .addTag(586, "K9",
                        6, DistanceUnit.INCH)
                .addCluster(clusterMetadata)
                .build();
    }

}
