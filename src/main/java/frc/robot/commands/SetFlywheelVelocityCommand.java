// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands;

import frc.robot.subsystems.FlywheelSubsystem;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

import edu.wpi.first.wpilibj2.command.Command;

public class SetFlywheelVelocityCommand extends Command {
    private final FlywheelSubsystem m_flywheelSubsystem;
    private final PIDController m_pidController;

    
    private final double m_targetVelocity;
    
    private SimpleMotorFeedforward m_feedforward;
    private double m_lastVelocity;

    public SetFlywheelVelocityCommand(FlywheelSubsystem subsystem, double targetVelocity) {
        m_flywheelSubsystem = subsystem;
        m_targetVelocity = targetVelocity;
        
        m_pidController = new PIDController(0, 0, 0);
        m_feedforward = new SimpleMotorFeedforward(0, 0, 0);

        

        addRequirements(m_flywheelSubsystem);
    }

    @Override
    public void initialize() {
        m_pidController.reset();
        m_lastVelocity = m_flywheelSubsystem.getFlywheelSpeed();
    }

    @Override
    public void execute() {
        double currentVelocity = m_flywheelSubsystem.getFlywheelSpeed();
        
        double pidOutput = m_pidController.calculate(currentVelocity, m_targetVelocity);
        double ffOutput = m_feedforward.calculateWithVelocities(m_lastVelocity, m_targetVelocity);
        
        m_flywheelSubsystem.setFlywheelVoltage(pidOutput + ffOutput);
        
        m_lastVelocity = currentVelocity;
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}