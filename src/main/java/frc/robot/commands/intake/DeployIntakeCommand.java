package frc.robot.commands.intake;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.Constants;

public class DeployIntakeCommand extends Command {

    public IntakeSubsystem intakeSubsystem;
    public final boolean runWheels;
    public DeployIntakeCommand(IntakeSubsystem intakeSubsystem, Supplier<Boolean> run) {
      this.intakeSubsystem = intakeSubsystem;
      this.runWheels = run.get();
      addRequirements(intakeSubsystem);
      }

    @Override
    public void initialize() {
        if (runWheels){
            intakeSubsystem.setIntakeWheelSpeed(Constants.IntakeConstants.WHEEL_SPEED);
        }

        intakeSubsystem.grabConstantsFromNetworkTables();
        intakeSubsystem.resetPIDController();
    }

    @Override
    public void execute() {
        double output = intakeSubsystem.closedLoopCalculate(Constants.IntakeConstants.DEPLOYED_ANGLE, intakeSubsystem.getPidController().getSetpoint().velocity);
        intakeSubsystem.setPivotVoltage(output);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
       return false;
    }
}
