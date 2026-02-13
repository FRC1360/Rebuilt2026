package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.Constants;

public class DeployIntakeCommand extends Command {

    private final IntakeSubsystem intakeSubsystem;

    public DeployIntakeCommand(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        intakeSubsystem.setIntakeWheelSpeed(Constants.IntakeConstants.WHEEL_SPEED);
        intakeSubsystem.grabConstantsFromNetworkTables();
        intakeSubsystem.resetPIDController();
    }

    @Override
    public void execute() {
        double output = intakeSubsystem.closedLoopCalculate(Constants.IntakeConstants.DEPLOYED_ANGLE, Constants.IntakeConstants.NEXT_VELOCITY);
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
