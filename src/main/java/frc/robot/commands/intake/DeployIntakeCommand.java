package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class DeployIntakeCommand extends Command {

    private static final double DEPLOY_ANGLE = 45.0;
    private static final double WHEEL_SPEED = 0.5;
    private static final double NEXT_VELOCITY = 0.0;

    private final IntakeSubsystem intakeSubsystem;

    public DeployIntakeCommand(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        intakeSubsystem.setIntakeWheelSpeed(WHEEL_SPEED);
    }

    @Override
    public void execute() {
        double output = intakeSubsystem.closedLoopCalculate(DEPLOY_ANGLE, NEXT_VELOCITY);
        intakeSubsystem.setPivotVoltage(output);

        intakeSubsystem.setIntakeWheelSpeed(WHEEL_SPEED);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
