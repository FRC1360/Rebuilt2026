package frc.robot.commands;

import static edu.wpi.first.units.Units.Rotation;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Set;

import org.photonvision.PhotonUtils;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.RotationTarget;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.util.FieldConstants;
import frc.robot.util.FieldZoneManager;
import frc.robot.util.RobotState;

public class TrenchRunCommands {

    private static final PathConstraints trenchRunConstraints = new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI);
    private static final NetworkTable loggingTable = NetworkTableInstance.getDefault().getTable("Commands/TrenchRun");
    private static final StructArrayPublisher<Pose2d> trajectoryPublisher = loggingTable
            .getStructArrayTopic("Goal Pose Iterations", Pose2d.struct).publish();

    private static List<Pose2d> neutral_to_blue_human = List.of(
            new Pose2d(FieldConstants.BLUE_HUMAN_TRENCH_NEUTRAL_ENTRY,
                    Rotation2d.k180deg),
            new Pose2d(FieldConstants.BLUE_HUMAN_TRENCH_ALLIANCE_ENTRY,
                    Rotation2d.k180deg));
    private static List<Pose2d> neutral_to_blue_depot = List.of(
            new Pose2d(FieldConstants.BLUE_DEPOT_TRENCH_NEUTRAL_ENTRY,
                    Rotation2d.k180deg),
            new Pose2d(FieldConstants.BLUE_DEPOT_TRENCH_ALLIANCE_ENTRY,
                    Rotation2d.k180deg));
    private static List<Pose2d> blue_to_neutral_human = List.of(
            new Pose2d(FieldConstants.BLUE_HUMAN_TRENCH_ALLIANCE_ENTRY,
                    Rotation2d.kZero),
            new Pose2d(FieldConstants.BLUE_HUMAN_TRENCH_NEUTRAL_ENTRY,
                    Rotation2d.kZero));
    private static List<Pose2d> blue_to_neutral_depot = List.of(
            new Pose2d(FieldConstants.BLUE_DEPOT_TRENCH_ALLIANCE_ENTRY,
                    Rotation2d.kZero),
            new Pose2d(FieldConstants.BLUE_DEPOT_TRENCH_NEUTRAL_ENTRY,
                    Rotation2d.kZero));

    private static List<Pose2d> returnBluePosesBasedOnRobotPose() {
        /* Initialize Util Classes */
        RobotState robotState = RobotState.getInstance();
        FieldZoneManager fieldZoneManager = FieldZoneManager.getInstance();

        /* Data from FZM */
        boolean inHuman = fieldZoneManager.isRobotInHumanPlayer();
        boolean inDepot = fieldZoneManager.isRobotInDepot();
        boolean inBlue = fieldZoneManager.isRobotInAlliance();
        boolean inNeutral = fieldZoneManager.isRobotInMiddle();

        /* Obtain robot positions in blue relative */
        Pose2d blueRelativeRobotPose = robotState.getRobotOdomPose();
        if (!robotState.isBlueAlliance.getAsBoolean())
            blueRelativeRobotPose = FlippingUtil.flipFieldPose(blueRelativeRobotPose);
        double robotRotationDegrees = MathUtil.inputModulus(
                blueRelativeRobotPose.getRotation().getDegrees(),
                -180, 180);

        /* Determine whether to compensate for intake going through trench */
        boolean offsetPathInPositiveX = false;
        boolean offsetPathInNegativeX = false;
        if (robotRotationDegrees <= -45 && robotRotationDegrees >= -135)
            offsetPathInPositiveX = true;
        else if (robotRotationDegrees >= 45 && robotRotationDegrees <= 135)
            offsetPathInNegativeX = true;

        /* Define output list based on FZM input */
        List<Pose2d> outputList = new ArrayList<>();
        if (inBlue && inHuman)
            outputList = blue_to_neutral_human;
        else if (inBlue && inDepot)
            outputList = blue_to_neutral_depot;
        else if (inNeutral && inHuman)
            outputList = neutral_to_blue_human;
        else if (inNeutral && inDepot)
            outputList = neutral_to_blue_depot;

        /* Implement Translational Compensation */
        if (offsetPathInPositiveX)
            outputList.forEach(pose -> new Pose2d(
                    pose.getTranslation().plus(FieldConstants.TRENCH_INTAKE_OFFSET),
                    pose.getRotation()));
        else if (offsetPathInNegativeX)
            outputList.forEach(pose -> new Pose2d(
                    pose.getTranslation().minus(FieldConstants.TRENCH_INTAKE_OFFSET),
                    pose.getRotation()));

        /* Find field relative heading towards entry pose + add to list */
        Rotation2d startingWaypointHeading = PhotonUtils.getYawToPose(
                new Pose2d(blueRelativeRobotPose.getTranslation(), new Rotation2d()),
                outputList.get(0));
        outputList.add(0, new Pose2d(blueRelativeRobotPose.getTranslation(), startingWaypointHeading));

        return outputList;
    }

    public static Command goToAllianceFromNeutralHumanCommand(CommandSwerveDrivetrain drivetrain) {

        RobotState robotState = RobotState.getInstance();

        return Commands.defer(() -> {
            Pose2d currentRobotPose = robotState.getRobotOdomPose();
            Pose2d startingPose, middlePose, endingPose;
            startingPose = new Pose2d();
            middlePose = new Pose2d();
            endingPose = new Pose2d();
            Rotation2d goalEndRotation = new Rotation2d();
            double goalEndVelocityMps = 2.0;

            Rotation2d robotRotation = currentRobotPose.getRotation();
            double robotRotationDegrees = MathUtil.inputModulus(robotRotation.getDegrees(), -180, 180);

            startingPose = new Pose2d(
                    currentRobotPose.getTranslation(),
                    PhotonUtils.getYawToPose(
                            new Pose2d(currentRobotPose.getTranslation(), new Rotation2d()),
                            new Pose2d(FieldConstants.BLUE_HUMAN_TRENCH_NEUTRAL_ENTRY, new Rotation2d())));

            if (!robotState.isBlueAlliance.getAsBoolean()) {
                currentRobotPose = FlippingUtil.flipFieldPose(currentRobotPose);
                startingPose = FlippingUtil.flipFieldPose(startingPose);
                robotRotation = currentRobotPose.getRotation();
                robotRotationDegrees = MathUtil.inputModulus(robotRotation.getDegrees(), -180, 180);
            }

            if (robotRotationDegrees <= -45 && robotRotationDegrees >= -135) {
                middlePose = new Pose2d(
                        FieldConstants.BLUE_HUMAN_TRENCH_NEUTRAL_ENTRY.plus(
                                FieldConstants.TRENCH_INTAKE_OFFSET),
                        Rotation2d.k180deg);
                endingPose = new Pose2d(
                        FieldConstants.BLUE_HUMAN_TRENCH_ALLIANCE_ENTRY.plus(
                                FieldConstants.TRENCH_INTAKE_OFFSET),
                        Rotation2d.k180deg);
                goalEndRotation = Rotation2d.kCW_90deg;
            } else if (robotRotationDegrees >= 45 && robotRotationDegrees <= 135) {
                middlePose = new Pose2d(
                        FieldConstants.BLUE_HUMAN_TRENCH_NEUTRAL_ENTRY.minus(
                                FieldConstants.TRENCH_INTAKE_OFFSET),
                        Rotation2d.k180deg);
                endingPose = new Pose2d(
                        FieldConstants.BLUE_HUMAN_TRENCH_ALLIANCE_ENTRY.minus(
                                FieldConstants.TRENCH_INTAKE_OFFSET),
                        Rotation2d.k180deg);
                goalEndRotation = Rotation2d.kCCW_90deg;
            } else {
                middlePose = new Pose2d(
                        FieldConstants.BLUE_HUMAN_TRENCH_NEUTRAL_ENTRY,
                        Rotation2d.k180deg);
                endingPose = new Pose2d(
                        FieldConstants.BLUE_HUMAN_TRENCH_ALLIANCE_ENTRY,
                        Rotation2d.k180deg);
                if ((robotRotationDegrees >= 90 && robotRotationDegrees <= 180) ||
                        (robotRotationDegrees <= -90 && robotRotationDegrees >= -180))
                    goalEndRotation = Rotation2d.k180deg;
                else
                    goalEndRotation = Rotation2d.kZero;
            }

            double distanceToTrench, distanceThroughTrench, totalDistanceTravelled;
            distanceToTrench = startingPose.getTranslation().getDistance(middlePose.getTranslation());
            distanceThroughTrench = middlePose.getTranslation().getDistance(endingPose.getTranslation());
            totalDistanceTravelled = distanceToTrench + distanceThroughTrench;
            RotationTarget rotationTargetAtTrenchEntry = new RotationTarget(
                    distanceToTrench / totalDistanceTravelled,
                    goalEndRotation);

            PathPlannerPath path = new PathPlannerPath(
                    PathPlannerPath.waypointsFromPoses(startingPose, middlePose, endingPose),
                    List.of(rotationTargetAtTrenchEntry),
                    Collections.emptyList(),
                    Collections.emptyList(),
                    Collections.emptyList(),
                    trenchRunConstraints,
                    null,
                    new GoalEndState(goalEndVelocityMps, goalEndRotation),
                    false);

            trajectoryPublisher.accept(path.getPathPoses().toArray(Pose2d[]::new));

            return AutoBuilder.followPath(path);
        }, Set.of(drivetrain));
    }

}
