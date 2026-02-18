package frc.robot.commands.intake;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.Constants;

public class RetractIntakeCommand extends Command {

    private final IntakeSubsystem intakeSubsystem;
    private final Supplier<Boolean> runWheels;


    public RetractIntakeCommand(IntakeSubsystem intakeSubsystem, Supplier<Boolean> runWheels) {
        this.intakeSubsystem = intakeSubsystem;
        this.runWheels = runWheels;
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

        if (runWheels.get()) {
    intakeSubsystem.setIntakeWheelSpeed(Constants.IntakeConstants.RETRACT_ANGLE);
        } else {
    intakeSubsystem.setIntakeWheelSpeed(0);
        }

    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}