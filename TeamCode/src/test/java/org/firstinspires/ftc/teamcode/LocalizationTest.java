
package org.firstinspires.ftc.teamcode;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import java.lang.reflect.Field;

import static org.junit.jupiter.api.Assertions.*;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.mockito.Mockito.*;

public class LocalizationTest {

    private Localization localization;

    private static final double TOLERANCE = 1e-6;

    @BeforeEach
    void setUp(){

        // Create a mock that executes real methods, but skips
        // the constructor (which requires robot hardware).
        localization = mock(Localization.class, CALLS_REAL_METHODS);
    }

    @Test public void testRotationCommandCalcs() {
        // Make sure ratio matches the measured result from earlier:
        assertEquals(0.2649, Localization.findRotateCommand(-1.0), 0.01, "Rotation calculation matches measured");

        // Make sure forward and backwards conversion between angular rate and command match:
        double[] testCases = {0, 0.5, -0.5, -1, 1};
        for (double rotate : testCases) {
            double rads = -Localization.calculateRotationRadPerSec(rotate);
            double result = Localization.findRotateCommand(rads);
            assertEquals(rotate, result, String.format("Rotating %.1f deg/s", Math.toDegrees(rads)));
        }
        // Intentionally clip:
        double rotate = 1.5;
        double rate = -Localization.calculateRotationRadPerSec(rotate);
        double result = Localization.findRotateCommand(rate);
        assertNotEquals(rotate, result, String.format("Rotating %.1f deg/s", Math.toDegrees(rate)));
    }

    @Test public void testSnapTo() {
        assertEquals(0, localization.getHeadingSnappedTo(90), TOLERANCE, "Snap 0 to 90");
        localization.setHeadingDeg(46);
        assertEquals(90, localization.getHeadingSnappedTo(90), TOLERANCE, "Snap 46 to 90");
        localization.setHeadingDeg(-44);
        assertEquals(0, localization.getHeadingSnappedTo(90), TOLERANCE, "Snap 46 to 90");
        localization.setHeadingDeg(-46);
        assertEquals(-90, localization.getHeadingSnappedTo(90), TOLERANCE, "Snap 46 to 90");
        localization.setHeadingDeg(179);
        assertEquals(180, localization.getHeadingSnappedTo(90), TOLERANCE, "Snap 179 to 90");
        localization.setHeadingDeg(-179);
        assertEquals(-180, localization.getHeadingSnappedTo(90), TOLERANCE, "Snap -179 to 90");
    }

    @Test public void testInDecelRadius(){
        localization.overrideRobotVelocity(10, 10);
        assertFalse( localization.inDecelRadius(10, 10, 9), "Outside decel radius");
        assertTrue( localization.inDecelRadius(7, 7, 10), "Inside decel radius");
    }

    @Test public void testDiffFromHeadingDeg(){
        double[][] testCases = {
                // Robot heading, target heading, difference
                {   0,    0,    0 },
                {   0,   90,   90 },
                {   0,  179,  179 },
                {   0,  -90,  -90 },
                {   0, -179, -179 },
                {   0,  350,  -10 },
                {   0, -350,   10 },
                {  90,   90,    0 } ,
                {  90,  181,   91 },
                {  90,  -91,  179 },
                { 179, -179,    2 },
                {-179,  179,   -2 },
                {-178, -179,   -1 },
                {-179, -178,    1 },
                { 178,  179,    1 },
                { 179,  178,   -1 },
        };

        for (double[] tc : testCases) {
            double headingDeg = tc[0];
            double targetDeg = tc[1];

            localization.setHeadingDeg(headingDeg);
            double result = localization.diffFromHeadingDeg(targetDeg);

            String message = String.format(
                    "Heading=%.0f°, Direction=%.0f°",
                    headingDeg, targetDeg
            );

            assertEquals(tc[2], result, TOLERANCE, message);
        }

    }



    /** Test all possible combinations of heading and direction through this function so it never hurts us again */
    @Test public void testConvertSpeedDirection() throws Exception {
        // {headingDeg, directionDeg, speed, expectedForward, expectedRight}
        double[][] testCases = {
                // Cardinal directions, heading = 0
                {   0,    0, 1.0,  1.0,  0.0 },
                {   0,   90, 1.0,  0.0, -1.0 },
                {   0,  180, 1.0, -1.0,  0.0 },
                {   0,  270, 1.0,  0.0,  1.0 },

                // Diagonal directions, heading = 0
                {   0,   45, 1.0,  0.70710678, -0.70710678 },
                {   0,  135, 1.0, -0.70710678, -0.70710678 },
                {   0,  225, 1.0, -0.70710678,  0.70710678 },
                {   0,  315, 1.0,  0.70710678,  0.70710678 },

                // Rotated robot headings
                {  90,    0, 1.0,  0.0,  1.0 },
                {  90,   90, 1.0,  1.0,  0.0 },
                { 180,    0, 1.0, -1.0,  0.0 },
                { 270,    0, 1.0,  0.0, -1.0 },
                {  45,   90, 1.0,  0.70710678, -0.70710678 },
                { 135,   45, 1.0,  0.0,  1.0 },

                // Negative angles and wraparound
                {   0,  -90, 1.0,  0.0,  1.0 },
                { -90,    0, 1.0,  0.0, -1.0 },
                { 315,   45, 1.0,  0.0, -1.0 },
                {  45,  315, 1.0,  0.0,  1.0 },
                { 360,    0, 1.0,  1.0,  0.0 },
                {   0,  360, 1.0,  1.0,  0.0 },

                // Speed scaling
                {   0,    0, 0.5,  0.5,  0.0 },
                {   0,   90, 0.5,  0.0, -0.5 },
                {  45,   90, 0.5,  0.35355339, -0.35355339 },

                // Zero and negative speed
                {  45,   90, 0.0,  0.0,  0.0 },
                {   0,    0,-1.0, -1.0,  0.0 },
                {   0,   90,-1.0,  0.0,  1.0 }
        };

        for (double[] tc : testCases) {
            double headingDeg = tc[0];
            double directionDeg = tc[1];

            localization.setHeadingDeg(headingDeg);
            localization.convertSpeedDirection(tc[2], Math.toRadians(directionDeg));

            String message = String.format(
                    "Heading=%.0f°, Direction=%.0f°, Speed=%.2f",
                    headingDeg, directionDeg, tc[2]
            );

            assertAll(message,
                    () -> assertEquals(tc[3], localization.newForward,
                            TOLERANCE, "Forward speed"),
                    () -> assertEquals(tc[4], localization.newRight,
                            TOLERANCE, "Right speed")
            );
        }
    }

    /** Make sure the field relative commands are working as desired */
    @Test public void testConvertFieldRelative() {
        localization.setHeadingDeg(0); // Facing right wall

        // Stick forward should strafe left
        localization.convertFieldRelative(1.0, 0, 0);
        assertAll("Stick forward, robot right, strafe left",
                () -> assertEquals(0, localization.newForward,
                        TOLERANCE, "Forward speed"),
                () -> assertEquals(-1, localization.newRight,
                        TOLERANCE, "Right speed")
        );

    }


}
