package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurretSubsystem;

public class TurnTurretToAngle extends Command {
    private final TurretSubsystem turretSubsystem;
    private double target;
    public TurnTurretToAngle(TurretSubsystem subsystem, double angle) {
      turretSubsystem = subsystem;
      target = angle;

      addRequirements(subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        turretSubsystem.setVoltage(target);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {}

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return false;
    }
}
