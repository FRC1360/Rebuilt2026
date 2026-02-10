// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.turret;

import java.util.function.Supplier;

import org.photonvision.PhotonUtils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.util.FieldConstants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AimTurretAtHubCommand extends Command {

    private final NetworkTable loggingTable = NetworkTableInstance.getDefault().getTable("Commands/"+getName());
    private final StructPublisher<Pose2d> turretPosePublisher = loggingTable.getStructTopic("Turret Pose", Pose2d.struct).publish();
    private final StructPublisher<Pose2d> robotPosePublisher = loggingTable.getStructTopic("Robot Pose", Pose2d.struct).publish();;
    private final StructPublisher<Pose2d> hubPosePublisher = loggingTable.getStructTopic("Hub Pose", Pose2d.struct).publish();;
    private final StructPublisher<Rotation2d> fieldRelativeTargetYawPublisher = loggingTable.getStructTopic("Target Yaw", Rotation2d.struct).publish();
    private final StructPublisher<Rotation2d> rotationSuppliedYaw = loggingTable.getStructTopic("Gyro Supplied Yaw", Rotation2d.struct).publish();


    private final Supplier<Pose2d> robotPoseSupplier;
    private final TurretSubsystem turretSubsystem;

    private Rotation2d targetFieldRelativeTurretRotation;
    private Rotation2d targetRobotRelativeTurretRotation;
    private Translation2d turretTranslation;
    private Translation2d hubTranslation;
    private Pose2d estimatedTurretPose;

    /** Creates a new AimTurretAtHub. */
    public AimTurretAtHubCommand(TurretSubsystem turretSubsystem, Supplier<Pose2d> robotPoseSupplier) {
        this.turretSubsystem = turretSubsystem;
        this.robotPoseSupplier = robotPoseSupplier;

        this.hubTranslation = FieldConstants.blueAllianceHubPose.getTranslation();

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(this.turretSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        estimatedTurretPose = turretSubsystem.getEstimatedPose();
        if (estimatedTurretPose == null) {
            Pose2d turretOnField = new Pose2d(
                robotPoseSupplier.get().getTranslation()
                    .plus(turretSubsystem.getRobotToTurret().getTranslation()),
                new Rotation2d()
            );
            estimatedTurretPose = turretOnField.rotateAround(
                turretOnField.getTranslation(),
                robotPoseSupplier.get().getRotation()
            );
        }
        // Rotation2d robotRotation = estimatedTurretPose.getRotation().minus(turretSubsystem.getCurrentRotation());
        Rotation2d robotRotation = robotPoseSupplier.get().getRotation();
        turretTranslation = estimatedTurretPose.getTranslation();
        
        targetFieldRelativeTurretRotation =
            PhotonUtils.getYawToPose(
                new Pose2d(turretTranslation, new Rotation2d()),
                new Pose2d(hubTranslation, new Rotation2d())
            );
        targetRobotRelativeTurretRotation = 
            targetFieldRelativeTurretRotation.minus(robotRotation);
        // targetRobotRelativeTurretRotation = targetFieldRelativeTurretRotation;
        turretSubsystem.setVoltage(
            turretSubsystem.closedLoopCalculate(targetRobotRelativeTurretRotation)
        );

        turretPosePublisher.accept(estimatedTurretPose);
        robotPosePublisher.accept(robotPoseSupplier.get());
        hubPosePublisher.accept(FieldConstants.blueAllianceHubPose);
        fieldRelativeTargetYawPublisher.accept(targetFieldRelativeTurretRotation);
        rotationSuppliedYaw.accept(targetRobotRelativeTurretRotation);
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
