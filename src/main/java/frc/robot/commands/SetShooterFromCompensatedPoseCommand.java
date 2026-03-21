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
import frc.robot.Constants.ShootingConstants;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.util.RobotState;
import frc.robot.util.ShootOnTheMoveOutputPoses;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetShooterFromCompensatedPoseCommand extends Command {

    private final NetworkTable loggingTable = NetworkTableInstance.getDefault().getTable("Commands/" + getName());
    private final StructPublisher<Pose2d> nonCompensatedGoalPosePublisher = loggingTable
            .getStructTopic("Non Compensated Goal Pose", Pose2d.struct).publish();
    private final StructPublisher<Pose2d> compensatedGoalPosePublisher = loggingTable
            .getStructTopic("Velocity Compensated Goal Pose", Pose2d.struct).publish();
    private final StructArrayPublisher<Pose2d> iteratedPosesPublisher = loggingTable
            .getStructArrayTopic("Goal Pose Iterations", Pose2d.struct).publish();

    private final RobotState robotState = RobotState.getInstance();

    private final TurretSubsystem turretSubsystem;
    private final HoodSubsystem hoodSubsystem;
    private final FlywheelSubsystem flywheelSubsystem;

    private Rotation2d targetFieldRelativeTurretRotation;
    private Rotation2d targetRobotRelativeTurretRotation;
    private Rotation2d robotRotation;
    private Translation2d turretTranslation;
    private Pose2d estimatedTurretPose;

    private ShootOnTheMoveOutputPoses shootOnTheMoveOutput;
    private Pose2d nonCompensatedGoalPose;
    private Translation2d compensatedGoalTranslation;
    private Pose2d[] iteratedPoses = new Pose2d[ShootingConstants.MAX_ITERATION_COUNT];

    /** Creates a new SetShooterFromCompensatedPoseCommand. */
    public SetShooterFromCompensatedPoseCommand(
            TurretSubsystem turretSubsystem,
            HoodSubsystem hoodSubsystem,
            FlywheelSubsystem flywheelSubsystem,
            Pose2d goalPose) {
        this.turretSubsystem = turretSubsystem;
        this.hoodSubsystem = hoodSubsystem;
        this.flywheelSubsystem = flywheelSubsystem;
        this.nonCompensatedGoalPose = goalPose;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(turretSubsystem, hoodSubsystem, flywheelSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        turretSubsystem.grabConstantsFromNetworkTables();
        turretSubsystem.resetPIDController();

        hoodSubsystem.grabConstantsFromNetworkTables();
        hoodSubsystem.resetPIDController();

        flywheelSubsystem.grabConstantsFromNetworkTables();
        flywheelSubsystem.resetPIDController();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // Calculate sotm output and store output poses
        shootOnTheMoveOutput = robotState.calculateParametersForShootOnTheMove(nonCompensatedGoalPose.getTranslation());
        compensatedGoalTranslation = shootOnTheMoveOutput.getGoalTranslation();
        for (int index = 0; index < ShootingConstants.MAX_ITERATION_COUNT; index++) {
            iteratedPoses[index] = new Pose2d(
                    shootOnTheMoveOutput.getIteratedTranslations()[index],
                    new Rotation2d());
        }

        // Gets the turretPosition, robot's rotation, and the turret translation from
        // the origin of the robot
        estimatedTurretPose = robotState.getTurretOdomPose();
        robotRotation = robotState.getRobotOdomPose().getRotation();
        turretTranslation = estimatedTurretPose.getTranslation();

        // Determines the angle between the turret's position and the Hub's position
        // (Field Relative Rotation).
        targetFieldRelativeTurretRotation = PhotonUtils.getYawToPose(
                new Pose2d(turretTranslation, new Rotation2d()),
                new Pose2d(compensatedGoalTranslation, new Rotation2d()));

        // Determines the robot felative rotation by subtracting the robot's current
        // position.
        targetRobotRelativeTurretRotation = targetFieldRelativeTurretRotation.minus(robotRotation);

        turretSubsystem.setVoltage(
                turretSubsystem.closedLoopCalculate(targetRobotRelativeTurretRotation));

        nonCompensatedGoalPosePublisher.accept(nonCompensatedGoalPose);
        compensatedGoalPosePublisher.accept(new Pose2d(compensatedGoalTranslation, new Rotation2d()));
        iteratedPosesPublisher.accept(iteratedPoses);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
