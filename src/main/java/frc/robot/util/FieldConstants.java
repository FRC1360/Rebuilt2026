package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class FieldConstants {
    public static final Pose2d RED_ALLIANCE_HUB_POSE = 
        new Pose2d(
            new Translation2d(16.54 - 4.625, 4.035),
            new Rotation2d()
        );
        
    public static final Pose2d BLUE_ALLIANCE_HUB_POSE = 
        new Pose2d(
            new Translation2d(4.625, 4.035),
            new Rotation2d()
        );

    public static final Pose2d BLUE_HUMAN_SIDE_PASS_POSE =
        new Pose2d(
            new Translation2d(3.0, 1.5),
            new Rotation2d()
        );
    public static final Pose2d BLUE_DEPOT_SIDE_PASS_POSE =
        new Pose2d(
            new Translation2d(3.0, (4.035 * 2.0) - 1.5),
            new Rotation2d()
        );

    public static final Pose2d RED_HUMAN_SIDE_PASS_POSE =
        new Pose2d(
            new Translation2d(16.54 - 3.0, (4.035 * 2.0) - 1.5),
            new Rotation2d()
        );
    public static final Pose2d RED_DEPOT_SIDE_PASS_POSE =
        new Pose2d(
            new Translation2d(16.54 - 3.0, 1.5),
            new Rotation2d()
        );
}