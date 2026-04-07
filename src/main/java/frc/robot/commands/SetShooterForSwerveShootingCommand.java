// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.photonvision.PhotonUtils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ShootingConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.util.RobotState;
import frc.robot.util.ShootOnTheMoveOutputPoses;

/**
 * Swerve-mode hub shot: sets hood + flywheel from SOTM-compensated distance
 * and drives the chassis to face the compensated heading. Does not require the
 * turret (which is held at home by its default command).
 */
public class SetShooterForSwerveShootingCommand extends Command {

    private static final double TRANSLATIONAL_SCALAR = 0.3;

    private final NetworkTable loggingTable = NetworkTableInstance.getDefault().getTable("Commands/" + getName());
    private final StructPublisher<Pose2d> nonCompensatedGoalPosePublisher = loggingTable
            .getStructTopic("Non Compensated Goal Pose", Pose2d.struct).publish();
    private final StructPublisher<Pose2d> compensatedGoalPosePublisher = loggingTable
            .getStructTopic("Velocity Compensated Goal Pose", Pose2d.struct).publish();
    private final StructArrayPublisher<Pose2d> iteratedPosesPublisher = loggingTable
            .getStructArrayTopic("Goal Pose Iterations", Pose2d.struct).publish();

    private final RobotState robotState = RobotState.getInstance();

    private final CommandSwerveDrivetrain drivetrain;
    private final CommandXboxController controller;
    private final HoodSubsystem hoodSubsystem;
    private final FlywheelSubsystem flywheelSubsystem;
    private final Pose2d nonCompensatedGoalPose;

    private ShootOnTheMoveOutputPoses shootOnTheMoveOutput;
    private Translation2d compensatedGoalTranslation;
    private Pose2d[] iteratedPoses = new Pose2d[ShootingConstants.MAX_ITERATION_COUNT];

    public SetShooterForSwerveShootingCommand(
            CommandSwerveDrivetrain drivetrain,
            CommandXboxController controller,
            HoodSubsystem hoodSubsystem,
            FlywheelSubsystem flywheelSubsystem,
            Pose2d goalPose) {
        this.drivetrain = drivetrain;
        this.controller = controller;
        this.hoodSubsystem = hoodSubsystem;
        this.flywheelSubsystem = flywheelSubsystem;
        this.nonCompensatedGoalPose = goalPose;

        addRequirements(drivetrain, hoodSubsystem, flywheelSubsystem);
    }

    @Override
    public void initialize() {
        hoodSubsystem.grabConstantsFromNetworkTables();
        hoodSubsystem.resetPIDController();

        flywheelSubsystem.grabConstantsFromNetworkTables();
        flywheelSubsystem.resetPIDController();
    }

    @Override
    public void execute() {
        shootOnTheMoveOutput = robotState.calculateParametersForShootOnTheMove(nonCompensatedGoalPose.getTranslation());
        compensatedGoalTranslation = shootOnTheMoveOutput.getGoalTranslation();
        for (int index = 0; index < ShootingConstants.MAX_ITERATION_COUNT; index++) {
            iteratedPoses[index] = new Pose2d(
                    shootOnTheMoveOutput.getIteratedTranslations()[index],
                    new Rotation2d());
        }

        Translation2d turretTranslation = robotState.getTurretOdomPose().getTranslation();
        Rotation2d compensatedHeading = PhotonUtils.getYawToPose(
                new Pose2d(turretTranslation, new Rotation2d()),
                new Pose2d(compensatedGoalTranslation, new Rotation2d()));

        Translation2d driveInput = DriveCommands.modifyJoystickCurve(
                -controller.getLeftY(), -controller.getLeftX());
        drivetrain.setControl(
                drivetrain.facingAngleRequest
                        .withVelocityX(driveInput.getX() * DriveCommands.MAX_DRIVE_TRANSLATIONAL_SPEED * TRANSLATIONAL_SCALAR)
                        .withVelocityY(driveInput.getY() * DriveCommands.MAX_DRIVE_TRANSLATIONAL_SPEED * TRANSLATIONAL_SCALAR)
                        .withTargetDirection(compensatedHeading));

        hoodSubsystem.setHoodMotorVoltage(
                hoodSubsystem.closedLoopCalculate(
                        robotState.getHoodAngleFromGoalPose(
                                new Pose2d(compensatedGoalTranslation, new Rotation2d()))));
        flywheelSubsystem.setFlywheelVoltage(
                flywheelSubsystem.closedLoopCalculate(
                        robotState.getFlywheelVelocityFromGoalPose(
                                new Pose2d(compensatedGoalTranslation, new Rotation2d()))));

        nonCompensatedGoalPosePublisher.accept(nonCompensatedGoalPose);
        compensatedGoalPosePublisher.accept(new Pose2d(compensatedGoalTranslation, new Rotation2d()));
        iteratedPosesPublisher.accept(iteratedPoses);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
