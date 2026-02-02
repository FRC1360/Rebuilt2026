// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HoodSubsystem;

public class SetHoodAngleCommand extends Command {

    /// PID constants
    private final double default_kP = 0.5;
    private final double default_kI = 1.0;
    private final double default_kD = 0.0;

    /// Motion profile limits
    private final double default_maxVelocity = 10.0;
    private final double default_maxAcceleration = 100.0;

    /// Subsystem and controllers
    private final HoodSubsystem m_hoodSubsystem;
    private final ProfiledPIDController m_hoodPidController;
    private final SimpleMotorFeedforward m_hoodFeedforward;

    /// State variables
    private double lastVelocity;
    private double targetAngleDegrees;
    private double pidControllerOutput;
    private double feedForwardControllerOutput;

    public SetHoodAngleCommand(HoodSubsystem hoodSubsystem, double targetAngleDegrees) {
        this.m_hoodSubsystem = hoodSubsystem;

        this.m_hoodPidController = new ProfiledPIDController(
            default_kP,
            default_kI,
            default_kD,
            new TrapezoidProfile.Constraints(default_maxVelocity, default_maxAcceleration)
        );
        /// Feedforward gains from sysid
        this.m_hoodFeedforward = new SimpleMotorFeedforward(
            0.159,
            0.012661,
            0.0
        );

        this.targetAngleDegrees = targetAngleDegrees;
        this.pidControllerOutput = 0.0;
        this.feedForwardControllerOutput = 0.0;

        addRequirements(m_hoodSubsystem);
    }

    @Override
    public void initialize() {
        /// Set PID gains and constraints
        m_hoodPidController.setP(default_kP);
        m_hoodPidController.setI(default_kI);
        m_hoodPidController.setD(default_kD);
        m_hoodPidController.setConstraints(
            new TrapezoidProfile.Constraints(default_maxVelocity, default_maxAcceleration)
        );
        m_hoodPidController.reset(
            m_hoodSubsystem.getCurrentAngle(),
            m_hoodSubsystem.getCurrentVelocity()
        );
        m_hoodPidController.setGoal(targetAngleDegrees);

        lastVelocity = m_hoodSubsystem.getCurrentVelocity();
    }

    @Override
    public void execute() {
        /// Calculate PID output
        pidControllerOutput = m_hoodPidController.calculate(m_hoodSubsystem.getCurrentAngle());
        /// Calculate feedforward
        feedForwardControllerOutput = m_hoodFeedforward.calculateWithVelocities(
            lastVelocity,
            m_hoodPidController.getSetpoint().velocity
        );

        /// Apply voltage
        m_hoodSubsystem.setHoodMotorVoltage(pidControllerOutput + feedForwardControllerOutput);

        lastVelocity = m_hoodSubsystem.getCurrentVelocity();
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}
