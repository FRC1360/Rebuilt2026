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
    /// Subsystem and controllers
    private final HoodSubsystem m_hoodSubsystem;


    private double targetAngleDegrees;

    public SetHoodAngleCommand(HoodSubsystem hoodSubsystem, double targetAngleDegrees) {
        this.m_hoodSubsystem = hoodSubsystem;
        this.targetAngleDegrees = targetAngleDegrees;
        addRequirements(m_hoodSubsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        m_hoodSubsystem.setHoodMotorVoltage(m_hoodSubsystem.closedLoopCalculate(targetAngleDegrees));
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}
