// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands;

import frc.robot.subsystems.FlywheelSubsystem;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

import edu.wpi.first.wpilibj2.command.Command;

public class SetFlywheelVelocityCommand extends Command {
    /// PID constants
    private final double default_kP = 0.05;
    private final double default_kI = 0.0;
    private final double default_kD = 0.003;
  
    /// Subsystem and controllers
    private final FlywheelSubsystem m_flywheelSubsystem;
    private final PIDController m_flyWheelPidController;
    private final SimpleMotorFeedforward m_flyWheelFeedForward;

    /// State variables
    private double lastFlywheelVelocity;
    private double targetFlyWheelVelocity;
    private double pidControllerOutput;
    private double feedForwardControllerOutput;

  public SetFlywheelVelocityCommand(FlywheelSubsystem subsystem, double targetVelocity) {
    m_flywheelSubsystem = subsystem;
    m_flyWheelPidController = new PIDController(default_kP, default_kI, default_kD);
    /// Feedforward gains from sysid
    m_flyWheelFeedForward = new SimpleMotorFeedforward(
      0.05901,
      0.10232,
      0.0
    );

    this.targetFlyWheelVelocity = targetVelocity;
    this.pidControllerOutput = 0.0;
    this.feedForwardControllerOutput = 0.0;
    addRequirements(m_flywheelSubsystem);
  }

  @Override
  public void initialize() {
    /// Set PID gains
    m_flyWheelPidController.setP(default_kP);
    m_flyWheelPidController.setI(default_kI);
    m_flyWheelPidController.setD(default_kD);
    m_flyWheelPidController.reset();

    lastFlywheelVelocity = m_flywheelSubsystem.getFlywheelSpeed();
  }

  @Override
  public void execute() {
    ///calculate the PID output
    pidControllerOutput = m_flyWheelPidController.calculate(m_flywheelSubsystem.getFlywheelSpeed(), targetFlyWheelVelocity);

    /// calculate feedforward
    feedForwardControllerOutput = m_flyWheelFeedForward.calculateWithVelocities(
            lastFlywheelVelocity,
            targetFlyWheelVelocity
    );
  
    /// apply voltage
    m_flywheelSubsystem.setFlywheelVoltage(pidControllerOutput + feedForwardControllerOutput);

    lastFlywheelVelocity = m_flywheelSubsystem.getFlywheelSpeed();
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
  
}