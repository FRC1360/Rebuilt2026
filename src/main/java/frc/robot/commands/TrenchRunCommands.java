package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;

import org.photonvision.PhotonUtils;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

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
import frc.robot.util.RobotState;

public class TrenchRunCommands {

    private static final PathConstraints trenchRunConstraints = new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI);
    private static final NetworkTable loggingTable = NetworkTableInstance.getDefault().getTable("Commands/TrenchRun");
    private static final StructArrayPublisher<Pose2d> trajectoryPublisher = loggingTable
            .getStructArrayTopic("Goal Pose Iterations", Pose2d.struct).publish();

    public static Command goToAllianceFromNeutralHumanCommand(CommandSwerveDrivetrain drivetrain) {

        RobotState robotState = RobotState.getInstance();

        return Commands.defer(() -> {
            Pose2d currentRobotPose = robotState.getRobotOdomPose();
            Pose2d startingPose, middlePose, endingPose;
            startingPose = new Pose2d();
            middlePose = new Pose2d();
            endingPose = new Pose2d();

            Rotation2d robotRotation = currentRobotPose.getRotation();
            double robotRotationDegrees = MathUtil.inputModulus(robotRotation.getDegrees(), -180, 180);

            if (robotState.getFieldRelativeRobotVelocityVector().getNorm() < 0.01) {
                startingPose = new Pose2d(
                        currentRobotPose.getTranslation(),
                        PhotonUtils.getYawToPose(
                                new Pose2d(currentRobotPose.getTranslation(), new Rotation2d()),
                                new Pose2d(FieldConstants.BLUE_HUMAN_TRENCH_NEUTRAL_ENTRY, new Rotation2d())));
            } else {
                startingPose = new Pose2d(
                        currentRobotPose.getTranslation(),
                        robotState.getFieldRelativeRobotVelocityVector().getAngle());
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
            } else if (robotRotationDegrees >= 45 && robotRotationDegrees <= 135) {
                middlePose = new Pose2d(
                        FieldConstants.BLUE_HUMAN_TRENCH_NEUTRAL_ENTRY.minus(
                                FieldConstants.TRENCH_INTAKE_OFFSET),
                        Rotation2d.k180deg);
                endingPose = new Pose2d(
                        FieldConstants.BLUE_HUMAN_TRENCH_ALLIANCE_ENTRY.minus(
                                FieldConstants.TRENCH_INTAKE_OFFSET),
                        Rotation2d.k180deg);
            } else {
                middlePose = new Pose2d(
                        FieldConstants.BLUE_HUMAN_TRENCH_NEUTRAL_ENTRY,
                        Rotation2d.k180deg);
                endingPose = new Pose2d(
                        FieldConstants.BLUE_HUMAN_TRENCH_ALLIANCE_ENTRY,
                        Rotation2d.k180deg);
            }

            PathPlannerPath path = new PathPlannerPath(
                    PathPlannerPath.waypointsFromPoses(startingPose, middlePose, endingPose),
                    trenchRunConstraints,
                    null,
                    new GoalEndState(0, Rotation2d.k180deg));

            trajectoryPublisher.accept(path.getPathPoses().toArray(Pose2d[]::new));

            return AutoBuilder.followPath(path);
        }, Set.of(drivetrain));
    }

}
