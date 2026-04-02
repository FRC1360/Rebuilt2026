package frc.robot.commands;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Set;
import java.util.stream.Collectors;

import org.photonvision.PhotonUtils;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.RotationTarget;
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
            .getStructArrayTopic("Trench Run Trajectory", Pose2d.struct).publish();

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

    public record TrenchRunResult(List<Pose2d> poses, Rotation2d roundedRotation) {
    }

    private static TrenchRunResult getBlueTrenchRunResult() {
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

        /* Create 'rounded' rotation values to snap to cardinal directions */
        Rotation2d roundedRotation;
        if (robotRotationDegrees > -45 && robotRotationDegrees <= 45)
            roundedRotation = Rotation2d.kZero;
        else if (robotRotationDegrees > 45 && robotRotationDegrees <= 135)
            roundedRotation = Rotation2d.fromDegrees(90);
        else if (robotRotationDegrees >= -135 && robotRotationDegrees <= -45)
            roundedRotation = Rotation2d.fromDegrees(-90);
        else
            roundedRotation = Rotation2d.k180deg;

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
            outputList = outputList.stream()
                    .map(p -> new Pose2d(p.getTranslation().plus(FieldConstants.TRENCH_INTAKE_OFFSET), p.getRotation()))
                    .collect(Collectors.toList());
        else if (offsetPathInNegativeX)
            outputList = outputList.stream()
                    .map(p -> new Pose2d(p.getTranslation().minus(FieldConstants.TRENCH_INTAKE_OFFSET),
                            p.getRotation()))
                    .collect(Collectors.toList());
        else
            outputList = new ArrayList<>(outputList);

        /* Find field relative heading towards entry pose + add to list */
        Rotation2d startingWaypointHeading = PhotonUtils.getYawToPose(
                new Pose2d(blueRelativeRobotPose.getTranslation(), new Rotation2d()),
                outputList.get(0));
        outputList.add(0, new Pose2d(blueRelativeRobotPose.getTranslation(), startingWaypointHeading));

        return new TrenchRunResult(outputList, roundedRotation);
    }

    public static Command autoTrenchRunCommand(CommandSwerveDrivetrain drivetrain) {
        return Commands.defer(() -> {
            /* Obtain Auto Pose List */
            TrenchRunResult trenchRunResult = getBlueTrenchRunResult();
            List<Pose2d> poseList = trenchRunResult.poses;
            Rotation2d snappedRotation = trenchRunResult.roundedRotation;

            /* Use straight-line distancing to approximate PP curve */
            double startToMiddleDistance = PhotonUtils.getDistanceToPose(
                    poseList.get(0), poseList.get(1));
            double middleToEndDistance = PhotonUtils.getDistanceToPose(
                    poseList.get(1), poseList.get(2));
            double rotationTargetPosition = startToMiddleDistance / (startToMiddleDistance + middleToEndDistance);

            /* Define rotation target */
            RotationTarget rotationTargetAtEntry = new RotationTarget(rotationTargetPosition, snappedRotation);

            /* Create path */            
            PathPlannerPath path = new PathPlannerPath(
                    PathPlannerPath.waypointsFromPoses(poseList),
                    List.of(rotationTargetAtEntry),
                    Collections.emptyList(),
                    Collections.emptyList(),
                    Collections.emptyList(),
                    trenchRunConstraints,
                    null,
                    new GoalEndState(3.0, snappedRotation),
                    false);

            trajectoryPublisher.accept(path.getPathPoses().toArray(Pose2d[]::new));
            return AutoBuilder.followPath(path);
        }, Set.of(drivetrain));
    }

}
