package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class FieldConstants {
    public static final Pose2d RED_ALLIANCE_HUB_POSE = new Pose2d(
            new Translation2d(16.54 - 4.625, 4.035),
            new Rotation2d());

    public static final Pose2d BLUE_ALLIANCE_HUB_POSE = new Pose2d(
            new Translation2d(4.625, 4.035),
            new Rotation2d());

    public static final Pose2d BLUE_HUMAN_SIDE_PASS_POSE = new Pose2d(
            new Translation2d(1.5, 3.0),
            new Rotation2d());
    public static final Pose2d BLUE_DEPOT_SIDE_PASS_POSE = new Pose2d(
            new Translation2d(1.5, (4.035 * 2.0) - 3.0),
            new Rotation2d());

    public static final Pose2d RED_HUMAN_SIDE_PASS_POSE = new Pose2d(
            new Translation2d(16.54 - 1.5, (4.035 * 2.0) - 3.0),
            new Rotation2d());
    public static final Pose2d RED_DEPOT_SIDE_PASS_POSE = new Pose2d(
            new Translation2d(16.54 - 1.5, 3.0),
            new Rotation2d());

    /* Used for TrenchRun */
    public static final double TRENCH_ENTRY_OFFSET_METERS = 1.0;
    public static final double TRENCH_WAYPOINTS_OFFSET_BY_INTAKE = 0.209 / 2.0; // Intake from bumper distance / 2
    public static final Translation2d TRENCH_INTAKE_OFFSET = new Translation2d(0.0, TRENCH_WAYPOINTS_OFFSET_BY_INTAKE);

    public static final Translation2d BLUE_HUMAN_TRENCH_CENTER = new Translation2d(
            Units.inchesToMeters(182.11),
            Units.inchesToMeters(25.335));
    public static final Translation2d BLUE_HUMAN_TRENCH_ALLIANCE_ENTRY = BLUE_HUMAN_TRENCH_CENTER
            .minus(new Translation2d(TRENCH_ENTRY_OFFSET_METERS, 0));
    public static final Translation2d BLUE_HUMAN_TRENCH_NEUTRAL_ENTRY = BLUE_HUMAN_TRENCH_CENTER
            .plus(new Translation2d(TRENCH_ENTRY_OFFSET_METERS, 0));

    public static final Translation2d BLUE_DEPOT_TRENCH_CENTER = new Translation2d(
            Units.inchesToMeters(182.11),
            Units.inchesToMeters(317.69 - 25.335));
    public static final Translation2d BLUE_DEPOT_TRENCH_ALLIANCE_ENTRY = BLUE_DEPOT_TRENCH_CENTER
            .minus(new Translation2d(TRENCH_ENTRY_OFFSET_METERS, 0));
    public static final Translation2d BLUE_DEPOT_TRENCH_NEUTRAL_ENTRY = BLUE_DEPOT_TRENCH_CENTER
            .plus(new Translation2d(TRENCH_ENTRY_OFFSET_METERS, 0));
}