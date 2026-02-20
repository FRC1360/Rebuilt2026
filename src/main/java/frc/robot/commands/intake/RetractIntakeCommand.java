package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.Constants.IntakeConstants;

public class RetractIntakeCommand extends Command {

    private final IntakeSubsystem intakeSubsystem;

    public RetractIntakeCommand(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        intakeSubsystem.setRollerSpeed(0.0);

        intakeSubsystem.grabConstantsFromNetworkTables();
        intakeSubsystem.resetPIDController();
    }

    @Override
    public void execute() {
        intakeSubsystem.setPivotVoltage(intakeSubsystem.closedLoopCalculate(IntakeConstants.PIVOT_RETRACTED_ANGLE));
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
