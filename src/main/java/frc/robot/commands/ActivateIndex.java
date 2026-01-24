package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexSubsystem;

public class ActivateIndex extends Command {
    IndexSubsystem indexSubsystem;
    double kVoltage;

    public ActivateIndex(IndexSubsystem subsystem, double volts) {
        indexSubsystem = subsystem;
        kVoltage = volts * 12;
        addRequirements(subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        indexSubsystem.setIndexVoltage(kVoltage);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {}

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        indexSubsystem.setIndexVoltage(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return false;
    }
}
