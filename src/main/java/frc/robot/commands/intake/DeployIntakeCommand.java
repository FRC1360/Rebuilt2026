package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.util.RobotState;
import frc.robot.Constants;

public class DeployIntakeCommand extends Command {

    private final RobotState robotState;

    public DeployIntakeCommand(RobotState robotState) {
        this.robotState = robotState;
        addRequirements(robotState);
            }
        
            private void addRequirements(RobotState robotState2) {
                // TODO Auto-generated method stub
                throw new UnsupportedOperationException("Unimplemented method 'addRequirements'");
            }
        
            @Override
    public void initialize() {
        robotState.setIntakeWheelSpeed(Constants.IntakeConstants.WHEEL_SPEED);
        robotState.grabConstantsFromNetworkTables();
        robotState.resetPIDController();
    }

    @Override
    public void execute() {
        double output = robotState.closedLoopCalculate(Constants.IntakeConstants.DEPLOYED_ANGLE, Constants.IntakeConstants.NEXT_VELOCITY);
        robotState.setPivotVoltage(output);

        robotState.setIntakeWheelSpeed(Constants.IntakeConstants.WHEEL_SPEED);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
