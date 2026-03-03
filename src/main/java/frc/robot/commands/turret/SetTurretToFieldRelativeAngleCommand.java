// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.turret;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.util.RobotState;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetTurretToFieldRelativeAngleCommand extends Command {

    private final NetworkTable loggingTable = NetworkTableInstance.getDefault().getTable("Subsystems/Gyro Testing");
    private final StructPublisher<Rotation2d> robotRotationPublisher = loggingTable.getStructTopic("Robot Yaw", Rotation2d.struct).publish();
    private final StructPublisher<Pose2d> turretPosePublisher = loggingTable.getStructTopic("Turret Pose", Pose2d.struct).publish();

    private final RobotState robotState = RobotState.getInstance();
    private final TurretSubsystem turretSubsystem;
    private final Rotation2d targetFieldRotation;

    private Rotation2d robotRotation;
    private Rotation2d targetTurretRotation;

    /** Creates a new SetFieldRelativeTurretRotation. */
    public SetTurretToFieldRelativeAngleCommand(
        TurretSubsystem turretSubsystem,
        Rotation2d targetFieldRotation
    ) {
        this.turretSubsystem = turretSubsystem;
        this.targetFieldRotation = targetFieldRotation;

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
        robotRotation = robotState.getRobotOdomPose().getRotation();

        robotRotationPublisher.accept(robotRotation);
        turretPosePublisher.accept(robotState.getTurretOdomPose());

        targetTurretRotation = targetFieldRotation.minus(robotRotation);
        turretSubsystem.setVoltage(
            turretSubsystem.closedLoopCalculate(targetTurretRotation)
        );
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
