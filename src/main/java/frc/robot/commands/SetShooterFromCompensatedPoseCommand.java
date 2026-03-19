// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.TurretSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetShooterFromCompensatedPoseCommand extends Command {

    private final NetworkTable loggingTable = NetworkTableInstance.getDefault().getTable("Commands/" + getName());
    private final StructPublisher<Pose2d> nonCompensatedGoalPosePublisher = 
        loggingTable.getStructTopic("Non Compensated Goal Pose", Pose2d.struct).publish();
    private final StructPublisher<Pose2d> compensatedGoalPosePublisher = 
        loggingTable.getStructTopic("Velocity Compensated Goal Pose", Pose2d.struct).publish();
    private final StructArrayPublisher<Pose2d> iteratedPosesPublisher = 
        loggingTable.getStructArrayTopic("Goal Pose Iterations", Pose2d.struct).publish();

    private final TurretSubsystem turretSubsystem;
    private final HoodSubsystem hoodSubsystem;
    private final FlywheelSubsystem flywheelSubsystem;

    private Pose2d goalPose;

    /** Creates a new SetShooterFromCompensatedPoseCommand. */
    public SetShooterFromCompensatedPoseCommand(
            TurretSubsystem turretSubsystem,
            HoodSubsystem hoodSubsystem,
            FlywheelSubsystem flywheelSubsystem,
            Pose2d goalPose) {
        this.turretSubsystem = turretSubsystem;
        this.hoodSubsystem = hoodSubsystem;
        this.flywheelSubsystem = flywheelSubsystem;
        this.goalPose = goalPose;

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
