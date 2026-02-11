package frc.robot.commands.index;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IndexSubsystem;

public class EnableHopperCommand extends Command {

    private final IndexSubsystem indexSubsystem;

    public EnableHopperCommand(IndexSubsystem indexSubsystem) {
        this.indexSubsystem = indexSubsystem;
        addRequirements(this.indexSubsystem);
    }

    @Override
    public void initialize() {
        indexSubsystem.setHopperSpeed(Constants.IndexConstants.HOPPER_SPEED);
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
