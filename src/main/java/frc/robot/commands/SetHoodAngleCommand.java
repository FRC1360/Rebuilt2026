// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HoodSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetHoodAngleCommand extends Command {

    private final double default_kP = 0.0;
    private final double default_kI = 0.0;
    private final double default_kD = 0.0;

    private final NetworkTable loggingTable;
    private final DoublePublisher hoodPIDOutputPublisher;
    private final DoublePublisher hoodSetpointPublisher;
    private final DoublePublisher hoodCurrentAnglePublisher;
    private final DoubleEntry kP_Entry;
    private final DoubleEntry kI_Entry;
    private final DoubleEntry kD_Entry;

    private final HoodSubsystem m_hoodSubsystem;
    private final PIDController m_hoodPidController;
    private final ArmFeedforward m_hoodFeedforward;

    private double targetAngleDegrees;
    private double pidControllerOutput;
    private double feedForwardControllerOutput;

    /** Creates a new SetHoodAngleCommand. */
    public SetHoodAngleCommand(HoodSubsystem hoodSubsystem, double targetAngleDegrees) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.m_hoodSubsystem = hoodSubsystem;

        this.m_hoodPidController = new PIDController(
            default_kP,
            default_kI,
            default_kD
        );
        this.m_hoodFeedforward = new ArmFeedforward(0.0, 0.0, 0.0, 0.0);

        loggingTable = NetworkTableInstance.getDefault().getTable("Commands/"+getName());
        hoodPIDOutputPublisher = loggingTable.getDoubleTopic("Hood PID Output").publish();
        hoodSetpointPublisher = loggingTable.getDoubleTopic("Hood Setpoint Degrees").publish();
        hoodCurrentAnglePublisher = loggingTable.getDoubleTopic("Hood Current Degrees").publish();
        kP_Entry = loggingTable.getDoubleTopic("kP").getEntry(default_kP);
        kI_Entry = loggingTable.getDoubleTopic("kI").getEntry(default_kI);
        kD_Entry = loggingTable.getDoubleTopic("kD").getEntry(default_kD);

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
        m_hoodPidController.reset();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        pidControllerOutput = m_hoodPidController.calculate(m_hoodSubsystem.getCurrentAngle(), targetAngleDegrees);
        feedForwardControllerOutput = m_hoodFeedforward.calculate(
        Units.degreesToRadians(m_hoodSubsystem.getCurrentAngle()),
        0.0
        );

        hoodPIDOutputPublisher.set(pidControllerOutput);
        hoodSetpointPublisher.set(targetAngleDegrees);
        hoodCurrentAnglePublisher.set(m_hoodSubsystem.getCurrentAngle());

        m_hoodSubsystem.setHoodMotorVoltage(pidControllerOutput + feedForwardControllerOutput);
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
