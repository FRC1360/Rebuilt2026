// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.AimTurretAtHub;
import frc.robot.commands.SetFieldRelativeTurretRotation;
import frc.robot.commands.SetRobotRelativeTurretRotation;
import frc.robot.subsystems.TurretSubsystem;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

    // // The robot's subsystems and commands are defined here...
    private final TurretSubsystem m_turretSubsystem = new TurretSubsystem();

    // // // Replace with CommandPS4Controller or CommandJoystick if needed
    private final CommandXboxController m_controller = new CommandXboxController(0);

    private final Pigeon2 m_pigeon2 = new Pigeon2(5);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        m_pigeon2.setYaw(0.0);

        // Configure the trigger bindings
        configureBindings();
    }

    private void configureBindings() {
        m_turretSubsystem.setDefaultCommand(
            new AimTurretAtHub(m_turretSubsystem, () -> Rotation2d.fromDegrees(m_pigeon2.getYaw().getValueAsDouble()))
        );

        m_controller.leftBumper().whileTrue(
            new SetFieldRelativeTurretRotation(() -> Rotation2d.fromDegrees(m_pigeon2.getYaw().getValueAsDouble()), m_turretSubsystem, Rotation2d.fromDegrees(-90))
        );
        m_controller.rightBumper().whileTrue(
            new SetFieldRelativeTurretRotation(() -> Rotation2d.fromDegrees(m_pigeon2.getYaw().getValueAsDouble()), m_turretSubsystem, Rotation2d.fromDegrees(90))
        );
        m_controller.leftTrigger(0.8).whileTrue(
            new SetFieldRelativeTurretRotation(() -> Rotation2d.fromDegrees(m_pigeon2.getYaw().getValueAsDouble()), m_turretSubsystem, Rotation2d.fromDegrees(-200))
        );
        m_controller.rightTrigger(0.8).whileTrue(
            new SetFieldRelativeTurretRotation(() -> Rotation2d.fromDegrees(m_pigeon2.getYaw().getValueAsDouble()), m_turretSubsystem, Rotation2d.fromDegrees(200))
        );

        m_controller.a().whileTrue(
            new SetFieldRelativeTurretRotation(
                () -> Rotation2d.fromDegrees(m_pigeon2.getYaw().getValueAsDouble()),
                m_turretSubsystem,
                new Rotation2d()
            )
        );
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An example command will be run in autonomous
        return null;
    }
}
