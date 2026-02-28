// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.flywheel;

import java.util.function.Supplier;

import org.photonvision.PhotonUtils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.Constants.FlywheelConstants;
/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetFlywheelFromPoseCommand extends Command {

    private final FlywheelSubsystem m_flywheelSubsystem;
    private final Pose2d m_targetPose;
    private final Pose2d turretPose; 
      
    /** Creates a new SetFlywheelToHubSpeedCommand. */
    public SetFlywheelFromPoseCommand(FlywheelSubsystem flywheelSubsystem, Pose2d targetPose, Supplier<Pose2d> turretPose) {
        m_flywheelSubsystem = flywheelSubsystem;
        m_targetPose = targetPose;
        this.turretPose = turretPose.get();
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(this.m_flywheelSubsystem);
    } 
     
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double distance = PhotonUtils.getDistanceToPose(this.turretPose, this.m_targetPose);
         if (distance > FlywheelConstants.DISTANCE_THRESHOLD) {
            m_flywheelSubsystem.setFlywheelSpeed(FlywheelConstants.HIGH_SPEED);
         } 
         else{
            m_flywheelSubsystem.setFlywheelSpeed(FlywheelConstants.LOW_SPEED);
         }
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
