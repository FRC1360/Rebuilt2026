// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.commands.SetFlywheelVelocityCommand;
import frc.robot.commands.SetHoodAngleCommand;
import frc.robot.subsystems.HoodSubsystem;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final FlywheelSubsystem m_flywheelSubsystem = new FlywheelSubsystem();
  private final HoodSubsystem m_hoodSubsystem = new HoodSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    m_hoodSubsystem.setDefaultCommand(
      new SetHoodAngleCommand(m_hoodSubsystem, 70)
    );

    m_flywheelSubsystem.setDefaultCommand(
      new InstantCommand(() -> m_flywheelSubsystem.setFlywheelVoltage(0), m_flywheelSubsystem)
    );

    m_driverController.a().whileTrue(
      new SetFlywheelVelocityCommand(m_flywheelSubsystem, 60)
    );
    m_driverController.x().whileTrue(
      new SetFlywheelVelocityCommand(m_flywheelSubsystem, 50)
    );
    m_driverController.b().whileTrue(
      new SetHoodAngleCommand(m_hoodSubsystem, 60)
    );

    // m_driverController.leftBumper().onTrue(Commands.runOnce(() -> DataLogManager.start()));
    // m_driverController.rightBumper().onTrue(Commands.runOnce(() -> DataLogManager.stop()));
    // m_driverController.rightTrigger(0.8).onTrue(Commands.runOnce(() -> System.out.println(DataLogManager.getLog())));

    // m_driverController.a().whileTrue(m_flywheelSubsystem.sysIdDynamic(Direction.kForward));
    // m_driverController.b().whileTrue(m_flywheelSubsystem.sysIdDynamic(Direction.kReverse));
    // m_driverController.x().whileTrue(m_flywheelSubsystem.sysIdQuasistatic(Direction.kForward));
    // m_driverController.y().whileTrue(m_flywheelSubsystem.sysIdQuasistatic(Direction.kReverse));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   * 
  
   */
  public Command getAutonomousCommand() {
    return null;   
  }
}
