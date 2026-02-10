// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.hood.SetHoodAngleCommand;
import frc.robot.subsystems.HoodSubsystem;

public class RobotContainer {

    //Replace with CommandPS4Controller or CommandJoystick if needed
    private final CommandXboxController m_controller = new CommandXboxController(0);

    private final HoodSubsystem m_hoodSubsystem = new HoodSubsystem();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        m_hoodSubsystem.setDefaultCommand(
            new RunCommand(() -> m_hoodSubsystem.setHoodMotorVoltage(0.0), m_hoodSubsystem)
        );

        m_controller.a().whileTrue(
            new SetHoodAngleCommand(m_hoodSubsystem, 70.0)
        );
        m_controller.b().whileTrue(
            new SetHoodAngleCommand(m_hoodSubsystem, 60.0)
        );
    }

    public Command getAutonomousCommand() {
        return null;
    }
}
