// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.flywheel.SetFlywheelVelocityCommand;
import frc.robot.commands.hood.SetHoodAngleCommand;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.HoodSubsystem;

public class RobotContainer {

    //Replace with CommandPS4Controller or CommandJoystick if needed
    private final CommandXboxController m_controller = new CommandXboxController(0);
    private static final RobotState robotState = RobotState.getInstance();

    private final HoodSubsystem m_HoodSubsystem = new HoodSubsystem();
    private final FlywheelSubsystem m_FlywheelSubsystem = new FlywheelSubsystem();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        m_HoodSubsystem.setDefaultCommand(
            new SetHoodAngleCommand(m_HoodSubsystem, 74)
        );
        m_FlywheelSubsystem.setDefaultCommand(
            new RunCommand(() -> m_FlywheelSubsystem.setFlywheelVoltage(0.0), m_FlywheelSubsystem)
        );

        m_controller.a().whileTrue(
            new SetHoodAngleCommand(m_HoodSubsystem, 70)
        );

        m_controller.b().whileTrue(
            new SetHoodAngleCommand(m_HoodSubsystem, 60)
        );

        m_controller.x().whileTrue(
            new SetHoodAngleCommand(m_HoodSubsystem, 50)
        );

        m_controller.leftBumper().whileTrue(
            new SetFlywheelVelocityCommand(m_FlywheelSubsystem, 40)
        );
        m_controller.leftTrigger(0.8).whileTrue(
            new SetFlywheelVelocityCommand(m_FlywheelSubsystem, 60)
        );
        m_controller.rightBumper().whileTrue(
            new SetFlywheelVelocityCommand(m_FlywheelSubsystem, 80)
        );
        m_controller.rightTrigger(0.8).whileTrue(
            new SetFlywheelVelocityCommand(m_FlywheelSubsystem, 100)
        );
    }

    public Command getAutonomousCommand() {
        return null;
    }
}
