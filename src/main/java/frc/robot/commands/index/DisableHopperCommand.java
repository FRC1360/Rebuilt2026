package frc.robot.commands.index;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexSubsystem;

public class DisableHopperCommand extends Command {

    private final IndexSubsystem indexSubsystem;

    public DisableHopperCommand(IndexSubsystem indexSubsystem) {
        this.indexSubsystem = indexSubsystem;
        addRequirements(this.indexSubsystem);
    }

    @Override
    public void initialize() {
        indexSubsystem.setHopperSpeed(0.0);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
