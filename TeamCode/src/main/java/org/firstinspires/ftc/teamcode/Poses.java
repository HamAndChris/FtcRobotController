package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class Poses {
    /** Left flower position */
    public static final Pose2D LEFT_FLOWER = new Pose2D(DistanceUnit.INCH,
            Calibration.AUTON_DIST_FLOWER_APPROACH, 48,
            AngleUnit.DEGREES, 180);

    /** Right (driver) flower position */
    public static final Pose2D POSE_RIGHT_FLOWER = new Pose2D(DistanceUnit.INCH,
            12 * 8, Calibration.AUTON_DIST_FLOWER_APPROACH,
            AngleUnit.DEGREES, -90);

    /** Left ideal firing position */
    public static final Pose2D LEFT_FIRING = new Pose2D(DistanceUnit.INCH,
            24, 48,
            AngleUnit.DEGREES, 0);

    /** Left ideal firing position */
    public static final Pose2D RIGHT_FIRING = new Pose2D(DistanceUnit.INCH,
            120, 48,
            AngleUnit.DEGREES, 180);
}
