package frc.robot.commands.intake;

import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.Constants.IntakeConstants;

public class DeployIntakeCommand extends Command {

    private final IntakeSubsystem intakeSubsystem;
    private final BooleanSupplier wheelsActivatedSupplier;

    public DeployIntakeCommand(IntakeSubsystem intakeSubsystem, BooleanSupplier wheelsActivatedSupplier) {
      this.intakeSubsystem = intakeSubsystem;
      this.wheelsActivatedSupplier = wheelsActivatedSupplier;
      addRequirements(intakeSubsystem);
      }

    @Override
    public void initialize() {
        intakeSubsystem.grabConstantsFromNetworkTables();
        intakeSubsystem.resetPIDController();
    }

    @Override
    public void execute() {
        intakeSubsystem.setPivotVoltage(intakeSubsystem.closedLoopCalculate(IntakeConstants.PIVOT_DEPLOYED_ANGLE));

        if (wheelsActivatedSupplier.getAsBoolean()) intakeSubsystem.setRollerSpeed(IntakeConstants.ROLLER_ACTIVATED_SPEED);
        else intakeSubsystem.setRollerSpeed(0.0);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
       return false;
    }
}
