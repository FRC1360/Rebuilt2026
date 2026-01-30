// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.TurnTurretToAngle;
import frc.robot.subsystems.TurretSubsystem;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final TurretSubsystem m_turretSubsystem = new TurretSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_controller =
      new CommandXboxController(0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

private void configureBindings() {
    // m_controller.b().whileTrue(m_turretSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // m_controller.a().whileTrue(m_turretSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // m_controller.x().whileTrue(m_turretSubsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    // m_controller.y().whileTrue(m_turretSubsystem.sysIdDynamic(SysIdRoutine.Direction.kForward));

    // m_controller.leftBumper().whileTrue(Commands.runOnce(() -> DataLogManager.start()));
    // m_controller.rightBumper().whileTrue(Commands.runOnce(() -> DataLogManager.stop()));

    // m_turretSubsystem.
    // setDefaultCommand(
    //   new RunCommand(() -> m_turretSubsystem.setVoltage(0.0), m_turretSubsystem)
    // );

    // m_controller.leftBumper().whileTrue(new TurnTurretToAngle(m_turretSubsystem, -160));
    // m_controller.rightBumper().whileTrue(new TurnTurretToAngle(m_turretSubsystem, 160));
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
