// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.hood;

import java.util.function.Supplier;

import org.photonvision.PhotonUtils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HoodSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AimHoodAtPoseCommand extends Command {

    private final HoodSubsystem hoodSubsystem;
    private Pose2d turretPose;
    private Pose2d PoseToAimAt; 
    private  InterpolatingDoubleTreeMap angletable = CreateMapCommand.turretPowerAngleDistancetable;
    

    /** Creates a new RetractHoodCommand. */
    public AimHoodAtPoseCommand(HoodSubsystem hoodSubsystem, Supplier<Pose2d> turretPose, Pose2d PoseToAimAt) {
        this.hoodSubsystem = hoodSubsystem;
        this.turretPose = turretPose.get();
        this.PoseToAimAt = PoseToAimAt;
        

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(this.hoodSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double distance = PhotonUtils.getDistanceToPose(this.turretPose, this.PoseToAimAt);
        double targetAngle = angletable.get(distance);
        hoodSubsystem.setHoodMotorVoltage(hoodSubsystem.closedLoopCalculate(targetAngle));
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
