package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class SetIntakePivotAngleCommand extends Command {

    private final IntakeSubsystem intakeSubsystem;
    private final double intakePivotAngle;
    private final double rollerSpeed;

    public SetIntakePivotAngleCommand(IntakeSubsystem intakeSubsystem, double intakePivotAngle, double rollerSpeed) {
      this.intakeSubsystem = intakeSubsystem;
      this.intakePivotAngle = intakePivotAngle;
      this.rollerSpeed = rollerSpeed;

      addRequirements(intakeSubsystem);
      }

    @Override
    public void initialize() {
        intakeSubsystem.grabConstantsFromNetworkTables();
        intakeSubsystem.resetPIDController();
    }

    @Override
    public void execute() {
        intakeSubsystem.setPivotVoltage(intakeSubsystem.closedLoopCalculate(intakePivotAngle));
        intakeSubsystem.setRollerSpeed(rollerSpeed);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
       return false;
    }
}
