// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands.flywheel;

import frc.robot.subsystems.FlywheelSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class SetFlywheelVoltageCommand extends Command {
    private final FlywheelSubsystem m_flywheelSubsystem;
    private final double m_voltage;

    public SetFlywheelVoltageCommand(FlywheelSubsystem subsystem, double voltage) {
        m_flywheelSubsystem = subsystem;
        m_voltage = voltage;
        addRequirements(m_flywheelSubsystem);
    }

    @Override
    public void initialize() {
        m_flywheelSubsystem.grabConstantsFromNetworkTables();
        m_flywheelSubsystem.resetPIDController();
    }

    @Override
    public void execute() {
        m_flywheelSubsystem.setFlywheelVoltage(m_voltage);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}