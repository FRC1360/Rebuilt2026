// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands.flywheel;

import frc.robot.subsystems.FlywheelSubsystem;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;

public class SetFlywheelVelocityFromNetworkTables extends Command {
    private final FlywheelSubsystem m_flywheelSubsystem;

    private final NetworkTable loggingTable;
    private final DoubleEntry flywheelSetpointEntry;

    public SetFlywheelVelocityFromNetworkTables(FlywheelSubsystem subsystem) {
        m_flywheelSubsystem = subsystem;

        loggingTable = NetworkTableInstance.getDefault().getTable("Commands/" + getName());
        flywheelSetpointEntry = loggingTable.getDoubleTopic("Flywheel Velocity Setpoint").getEntry(60);
        flywheelSetpointEntry.set(60);

        addRequirements(m_flywheelSubsystem);
    }

    @Override
    public void initialize() {
        m_flywheelSubsystem.grabConstantsFromNetworkTables();
        m_flywheelSubsystem.resetPIDController();
    }

    @Override
    public void execute() {
        m_flywheelSubsystem.setFlywheelVoltage(m_flywheelSubsystem.closedLoopCalculate(flywheelSetpointEntry.get()));
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}