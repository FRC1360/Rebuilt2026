// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HoodSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetHoodAngleCommand extends Command {

  private final HoodSubsystem m_hoodSubsystem;
  private final PIDController m_hoodPidController;
  private final ArmFeedforward m_hoodFeedforward;
  private double targetAngleDegrees;

  /** Creates a new SetHoodAngleCommand. */
  public SetHoodAngleCommand(HoodSubsystem hoodSubsystem, double targetAngleDegrees) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_hoodSubsystem = hoodSubsystem;
    this.m_hoodPidController = new PIDController(0.1175, 0.1, 0.0);
    this.m_hoodFeedforward = new ArmFeedforward(0.0, 0.5, 0.0, 0.0);
    this.targetAngleDegrees = targetAngleDegrees;

    addRequirements(m_hoodSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // m_hoodPidController.setP(SmartDashboard.getNumber("Commands/SetHoodAngleCommand/Hood PID Kp", 0.0));
    // m_hoodPidController.setI(SmartDashboard.getNumber("Commands/SetHoodAngleCommand/Hood PID Ki", 0.0));
    // m_hoodPidController.setD(SmartDashboard.getNumber("Commands/SetHoodAngleCommand/Hood PID Kd", 0.0));
    m_hoodPidController.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // SmartDashboard.putData("Commands/SetHoodAngleCommand/Hood PID Controller", m_hoodPidController);
    m_hoodPidController.setSetpoint(targetAngleDegrees);

    SmartDashboard.putNumber("Commands/SetHoodAngleCommand/Hood PID Output", m_hoodPidController.calculate(m_hoodSubsystem.getCurrentAngle(), targetAngleDegrees));
    SmartDashboard.putNumber("Commands/SetHoodAngleCommand/Hood PID Error", m_hoodPidController.getError());
    SmartDashboard.putNumber("Commands/SetHoodAngleCommand/Hood PID Setpoint", targetAngleDegrees);

    m_hoodSubsystem.setHoodMotorVoltage(
      m_hoodPidController.calculate(m_hoodSubsystem.getCurrentAngle(), targetAngleDegrees)
    );

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
