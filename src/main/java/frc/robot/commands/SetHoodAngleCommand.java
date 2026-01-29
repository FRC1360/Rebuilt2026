// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HoodSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetHoodAngleCommand extends Command {

    private final double default_kP = 0.5;
    private final double default_kI = 1.0;
    private final double default_kD = 0.0;

    private final double default_maxVelocity = 10.0;
    private final double default_maxAcceleration = 100.0;

    private final NetworkTable loggingTable;
    private final DoublePublisher hoodPIDOutputPublisher;
    private final DoublePublisher hoodSetpointPublisher;
    private final DoublePublisher hoodCurrentAnglePublisher;
    private final DoubleEntry kP_Entry;
    private final DoubleEntry kI_Entry;
    private final DoubleEntry kD_Entry;
    private final DoubleEntry maxAcceleration_Entry;
    private final DoubleEntry maxVelocity_Entry;

    private final HoodSubsystem m_hoodSubsystem;
    private final ProfiledPIDController m_hoodPidController;
    private final SimpleMotorFeedforward m_hoodFeedforward;

    private double lastVelocity;
    private double targetAngleDegrees;
    private double pidControllerOutput;
    private double feedForwardControllerOutput;

    /** Creates a new SetHoodAngleCommand. */
    public SetHoodAngleCommand(HoodSubsystem hoodSubsystem, double targetAngleDegrees) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.m_hoodSubsystem = hoodSubsystem;

        this.m_hoodPidController = new ProfiledPIDController(
            default_kP,
            default_kI,
            default_kD,
            new TrapezoidProfile.Constraints(default_maxVelocity, default_maxAcceleration)
        );
        this.m_hoodFeedforward = new SimpleMotorFeedforward(
            0.159,
            0.012661,
            0.0
        );

        loggingTable = NetworkTableInstance.getDefault().getTable("Commands/"+getName());
        hoodPIDOutputPublisher = loggingTable.getDoubleTopic("Hood PID Output").publish();
        hoodSetpointPublisher = loggingTable.getDoubleTopic("Hood Setpoint Degrees").publish();
        hoodCurrentAnglePublisher = loggingTable.getDoubleTopic("Hood Current Degrees").publish();
        kP_Entry = loggingTable.getDoubleTopic("kP").getEntry(default_kP);
        kI_Entry = loggingTable.getDoubleTopic("kI").getEntry(default_kI);
        kD_Entry = loggingTable.getDoubleTopic("kD").getEntry(default_kD);
        maxVelocity_Entry = loggingTable.getDoubleTopic("Max Velocity").getEntry(default_maxVelocity);
        maxAcceleration_Entry = loggingTable.getDoubleTopic("Max Acceleration").getEntry(default_maxAcceleration);

        kP_Entry.set(default_kP);
        kI_Entry.set(default_kI);
        kD_Entry.set(default_kD);

        this.targetAngleDegrees = targetAngleDegrees;
        this.pidControllerOutput = 0.0;
        this.feedForwardControllerOutput = 0.0;

        addRequirements(m_hoodSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_hoodPidController.setP(kP_Entry.get());
        m_hoodPidController.setI(kI_Entry.get());
        m_hoodPidController.setD(kD_Entry.get());
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

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        
        pidControllerOutput = m_hoodPidController.calculate(m_hoodSubsystem.getCurrentAngle());
        feedForwardControllerOutput = m_hoodFeedforward.calculateWithVelocities(
            lastVelocity,
            m_hoodPidController.getSetpoint().velocity
        );

        hoodPIDOutputPublisher.set(feedForwardControllerOutput);
        hoodSetpointPublisher.set(m_hoodPidController.getSetpoint().position);
        hoodCurrentAnglePublisher.set(m_hoodSubsystem.getCurrentAngle());

        m_hoodSubsystem.setHoodMotorVoltage(pidControllerOutput + feedForwardControllerOutput);

        lastVelocity = m_hoodSubsystem.getCurrentVelocity();
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
