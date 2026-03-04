// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.hood;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.util.RobotState;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetHoodAngleFromPose extends Command {

    private final HoodSubsystem hoodSubsystem;
    private final RobotState robotState = RobotState.getInstance();
    private Pose2d poseToAimAt;

    private final NetworkTable loggingTable;
    private final DoublePublisher distanceToTargetPublisher;
    private final StructPublisher<Pose2d> turretPosePublisher;
    private final StructPublisher<Pose2d> goalPosePublisher;

    /** Creates a new RetractHoodCommand. */
    public SetHoodAngleFromPose(HoodSubsystem hoodSubsystem, Pose2d PoseToAimAt) {
        this.hoodSubsystem = hoodSubsystem;
        this.poseToAimAt = PoseToAimAt;

        loggingTable = NetworkTableInstance.getDefault().getTable("Commands/" + getName());
        distanceToTargetPublisher = loggingTable.getDoubleTopic("Distance To Goal").publish();
        turretPosePublisher = loggingTable.getStructTopic("Turret Pose", Pose2d.struct).publish();
        goalPosePublisher = loggingTable.getStructTopic("Goal Pose", Pose2d.struct).publish();

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(this.hoodSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        hoodSubsystem.grabConstantsFromNetworkTables();
        hoodSubsystem.resetPIDController();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        hoodSubsystem.setHoodMotorVoltage(hoodSubsystem.closedLoopCalculate(
                robotState.getHoodAngleFromGoalPose(poseToAimAt)));

        // Update publishers
        distanceToTargetPublisher.accept(
                poseToAimAt.getTranslation().getDistance(robotState.getTurretOdomPose().getTranslation()));
        turretPosePublisher.accept(robotState.getTurretOdomPose());
        goalPosePublisher.accept(poseToAimAt);
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
