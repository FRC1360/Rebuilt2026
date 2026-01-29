// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands;

import frc.robot.subsystems.FlywheelSubsystem;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

import edu.wpi.first.math.util.Units;

import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */

public class SetFlywheelVelocityCommand extends Command {
    private final double default_kP = 0.05;
    private final double default_kI = 0.0;
    private final double default_kD = 0.003;

    private final NetworkTable loggingTable;
    private final DoublePublisher flyWheelVoltageOutputPublisher;
    private final DoublePublisher flyWheelSetTargetVelocityPublisher;
    private final DoublePublisher flyWheelVelocityPublisher;
    private final DoubleEntry kP_Entry;
    private final DoubleEntry kI_Entry;
    private final DoubleEntry kD_Entry;
  
    private final FlywheelSubsystem m_flywheelSubsystem;
    private final PIDController m_flyWheelPidController;
    private final SimpleMotorFeedforward m_flyWheelFeedForward;

    private double lastFlywheelVelocity;
    private double targetFlyWheelVelocity;
    private double pidControllerOutput;
    private double feedForwardControllerOutput;
    
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */

  public SetFlywheelVelocityCommand(FlywheelSubsystem subsystem, double targetVelocity) {
    m_flywheelSubsystem = subsystem;
    m_flyWheelPidController = new PIDController(default_kP, default_kI, default_kD);
    m_flyWheelFeedForward = new SimpleMotorFeedforward(
      0.05901,
      0.10232,
      0.0
    );

    loggingTable = NetworkTableInstance.getDefault().getTable("Commands/" + getName());
    flyWheelVoltageOutputPublisher = loggingTable.getDoubleTopic("FlyWheel PID + FF Output").publish(); // fix later
    flyWheelSetTargetVelocityPublisher = loggingTable.getDoubleTopic("Flywheel target velocity").publish();
    flyWheelVelocityPublisher = loggingTable.getDoubleTopic("FlyWheel current velocity").publish();

    kP_Entry = loggingTable.getDoubleTopic("KP").getEntry(default_kP);
    kI_Entry = loggingTable.getDoubleTopic("KI").getEntry(default_kI);
    kD_Entry = loggingTable.getDoubleTopic("KD").getEntry(default_kD);

    kP_Entry.set(default_kP);
    kI_Entry.set(default_kI);
    kD_Entry.set(default_kD);

    this.targetFlyWheelVelocity = targetVelocity;
    this.pidControllerOutput = 0.0;
    this.feedForwardControllerOutput = 0.0;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_flywheelSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_flyWheelPidController.setP(kP_Entry.get());
    m_flyWheelPidController.setI(kI_Entry.get());
    m_flyWheelPidController.setD(kD_Entry.get());
    m_flyWheelPidController.reset();

    lastFlywheelVelocity = m_flywheelSubsystem.getFlywheelSpeed();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pidControllerOutput = m_flyWheelPidController.calculate(m_flywheelSubsystem.getFlywheelSpeed(), targetFlyWheelVelocity);

    feedForwardControllerOutput = m_flyWheelFeedForward.calculateWithVelocities(
            lastFlywheelVelocity,
            targetFlyWheelVelocity
    );

    flyWheelVoltageOutputPublisher.set(pidControllerOutput + feedForwardControllerOutput);
    flyWheelSetTargetVelocityPublisher.set(targetFlyWheelVelocity);
    flyWheelVelocityPublisher.set(m_flywheelSubsystem.getFlywheelSpeed());
  
    m_flywheelSubsystem.setFlywheelVoltage(pidControllerOutput + feedForwardControllerOutput);

    lastFlywheelVelocity = m_flywheelSubsystem.getFlywheelSpeed();
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}
  // Returns true when the command should end.

  @Override
  public boolean isFinished() {
    return false;
  }
  
}