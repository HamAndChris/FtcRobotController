
package org.firstinspires.ftc.teamcode;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import java.lang.reflect.Field;

import static org.junit.jupiter.api.Assertions.*;
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

    /** Test all possible combinations of heading and direction through this function so it never hurts us again */
    @Test void testConvertSpeedDirection() throws Exception {
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
