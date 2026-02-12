// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.turret;

import org.photonvision.PhotonUtils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.util.RobotState;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AimTurretAtPoseCommand extends Command {

    private final NetworkTable loggingTable = NetworkTableInstance.getDefault().getTable("Commands/"+getName());
    private final StructPublisher<Pose2d> turretPosePublisher = loggingTable.getStructTopic("Turret Pose", Pose2d.struct).publish();
    private final StructPublisher<Pose2d> robotPosePublisher = loggingTable.getStructTopic("Robot Pose", Pose2d.struct).publish();
    private final StructPublisher<Pose2d> goalPosePublisher = loggingTable.getStructTopic("Goal Pose", Pose2d.struct).publish();
    private final StructPublisher<Rotation2d> fieldRelativeGoalYaw = loggingTable.getStructTopic("Field Relative Goal Yaw", Rotation2d.struct).publish();

    private final RobotState robotState = RobotState.getInstance();

    private final TurretSubsystem turretSubsystem;

    private Rotation2d targetFieldRelativeTurretRotation;
    private Rotation2d targetRobotRelativeTurretRotation;
    private Rotation2d robotRotation;
    private Translation2d turretTranslation;
    private Pose2d estimatedTurretPose;
    private Pose2d goalPose;

    /** Creates a new AimTurretAtHub. */
    public AimTurretAtPoseCommand(TurretSubsystem turretSubsystem, Pose2d goalPose) {
        this.turretSubsystem = turretSubsystem;
        this.goalPose = goalPose;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(this.turretSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        turretSubsystem.grabConstantsFromNetworkTables();
        turretSubsystem.resetPIDController();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        estimatedTurretPose = robotState.getTurretOdomPose();
        robotRotation = robotState.getRobotOdomPose().getRotation();
        turretTranslation = estimatedTurretPose.getTranslation();
        
        targetFieldRelativeTurretRotation =
            PhotonUtils.getYawToPose(
                new Pose2d(turretTranslation, new Rotation2d()),
                new Pose2d(goalPose.getTranslation(), new Rotation2d())
            );
        targetRobotRelativeTurretRotation = 
            targetFieldRelativeTurretRotation.minus(robotRotation);

        turretSubsystem.setVoltage(
            turretSubsystem.closedLoopCalculate(targetRobotRelativeTurretRotation)
        );
        
        turretPosePublisher.accept(estimatedTurretPose);
        robotPosePublisher.accept(robotState.getRobotOdomPose());
        goalPosePublisher.accept(goalPose);
        fieldRelativeGoalYaw.accept(targetFieldRelativeTurretRotation);
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
