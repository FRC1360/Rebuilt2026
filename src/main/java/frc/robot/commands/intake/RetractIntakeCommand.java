package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.Constants;

public class RetractIntakeCommand extends Command {

    private final IntakeSubsystem intakeSubsystem;

    public RetractIntakeCommand(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        intakeSubsystem.grabConstantsFromNetworkTables();
        intakeSubsystem.resetPIDController();
    }

    @Override
    public void execute() {
        double output = intakeSubsystem.closedLoopCalculate(Constants.IntakeConstants.RETRACT_ANGLE, intakeSubsystem.getPidController().getSetpoint().velocity);
        intakeSubsystem.setPivotVoltage(output);

        intakeSubsystem.setIntakeWheelSpeed(Constants.IntakeConstants.WHEEL_SPEED);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
