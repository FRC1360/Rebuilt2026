// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurretSubsystem;

import static edu.wpi.first.units.Units.Degrees;
import com.ctre.phoenix6.hardware.Pigeon2;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TurnTurretToGyroRelative extends Command {

    private final double default_kP = 0.055;
    private final double default_kI = 0.01;
    private final double default_kD = 0.0;
    private final double default_maxVelocity = 3000;
    private final double default_maxAcceleration = 3500;
    private final double default_kS = 0.11;
    private final double default_kV = 0.0022;
    private final double default_kA = 0.0;
    private final int pigeonDeviceID = 6;

    private final NetworkTable loggingTable;
    private final DoublePublisher controlLoopOutputPublisher;
    private final DoublePublisher setpointPublisher;
    private final DoublePublisher setpointVelocityPublisher;
    private final DoublePublisher currentAnglePublisher;
    private final DoublePublisher currentVelocityPublisher;
    private final DoublePublisher pigeonYawPublisher;

    private final DoubleEntry kP_Entry;
    private final DoubleEntry kI_Entry;
    private final DoubleEntry kD_Entry;
    private final DoubleEntry maxAcceleration_Entry;
    private final DoubleEntry maxVelocity_Entry;
    private final DoubleEntry kS_Entry;
    private final DoubleEntry kV_Entry;
    private final DoubleEntry kA_Entry;
    private final DoubleEntry pigeonYawEntry;

    private final TurretSubsystem m_TurretSubsystem;
    // private final ProfiledPIDController m_pidController;
    // private final SimpleMotorFeedforward m_feedForward;

    private double lastVelocity;
    // private double pidControllerOutput;
    // private double feedForwardControllerOutput;

    private Pigeon2 pigeon;
    private double pigeonYaw;

    /** Creates a new SetHoodAngleCommand. */
    public TurnTurretToGyroRelative(TurretSubsystem turretSubsystem) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.m_TurretSubsystem = turretSubsystem;
        this.pigeon = new Pigeon2(pigeonDeviceID);

        // this.m_pidController = new ProfiledPIDController(
        //     default_kP,
        //     default_kI,
        //     default_kD,
        //     new TrapezoidProfile.Constraints(default_maxVelocity, default_maxAcceleration)
        // );
        // this.m_feedForward = new SimpleMotorFeedforward(
        //     default_kS,
        //     default_kV,
        //     default_kA
        // );

        loggingTable = NetworkTableInstance.getDefault().getTable("Commands/"+getName());
        controlLoopOutputPublisher = loggingTable.getDoubleTopic("PID Output").publish();
        setpointPublisher = loggingTable.getDoubleTopic("Setpoint Degrees").publish();
        currentAnglePublisher = loggingTable.getDoubleTopic("Current Degrees").publish();
        currentVelocityPublisher = loggingTable.getDoubleTopic("Current Velocity Degrees per Sec").publish();
        setpointVelocityPublisher = loggingTable.getDoubleTopic("Setpoint Velocity Degrees per Sec").publish();
        pigeonYawPublisher = loggingTable.getDoubleTopic("Pigeon Yaw in degress").publish();

        kP_Entry = loggingTable.getDoubleTopic("kP").getEntry(default_kP);
        kI_Entry = loggingTable.getDoubleTopic("kI").getEntry(default_kI);
        kD_Entry = loggingTable.getDoubleTopic("kD").getEntry(default_kD);
        maxVelocity_Entry = loggingTable.getDoubleTopic("Max Velocity").getEntry(default_maxVelocity);
        maxAcceleration_Entry = loggingTable.getDoubleTopic("Max Acceleration").getEntry(default_maxAcceleration);
        kS_Entry = loggingTable.getDoubleTopic("kS").getEntry(default_kS);
        kV_Entry = loggingTable.getDoubleTopic("kV").getEntry(default_kV);
        kA_Entry = loggingTable.getDoubleTopic("kA").getEntry(default_kA);
        pigeonYawEntry = loggingTable.getDoubleTopic("Pigeon Yaw").getEntry(pigeonYaw);


        kP_Entry.set(default_kP);
        kI_Entry.set(default_kI);
        kD_Entry.set(default_kD);
        maxVelocity_Entry.set(default_maxVelocity);
        maxAcceleration_Entry.set(default_maxAcceleration);
        kS_Entry.set(default_kS);
        kV_Entry.set(default_kV);
        kA_Entry.set(default_kA);
        pigeonYawEntry.set(pigeonYaw);

        //this.targetAngleDegrees = targetAngleDegrees;
        // this.pidControllerOutput = 0.0;
        // this.feedForwardControllerOutput = 0.0;

        //reset pigeon yaw on startup
        this.pigeon.setYaw(0.0);
    
        addRequirements(m_TurretSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // m_pidController.setP(kP_Entry.get());
        // m_pidController.setI(kI_Entry.get());
        // m_pidController.setD(kD_Entry.get());
        // m_pidController.setConstraints(
        //     new TrapezoidProfile.Constraints(
        //         maxVelocity_Entry.get(), 
        //         maxAcceleration_Entry.get()
        //     )
        // );
        // m_pidController.reset(
        //     m_TurretSubsystem.getCurrentAngle(),
        //     m_TurretSubsystem.getCurrentVelocity()
        // );

        // m_feedForward.setKs(kS_Entry.get());
        // m_feedForward.setKv(kV_Entry.get());
        // m_feedForward.setKa(kA_Entry.get());

        lastVelocity = m_TurretSubsystem.getCurrentVelocity();
    }


    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // controlLoopOutputPublisher.set(m_TurretSubsystem.getTurretFeedforwardOutput());
        // setpointPublisher.set(m_TurretSubsystem.getWrappedTarget());
        // setpointVelocityPublisher.set(m_TurretSubsystem.getTurretPIDController().getSetpoint().velocity);
        // currentAnglePublisher.set(m_TurretSubsystem.getCurrentAngle());
        // currentVelocityPublisher.set(m_TurretSubsystem.getCurrentVelocity());

        pigeonYaw = -pigeon.getYaw().getValue().in(Degrees);
        Rotation2d pigeonYawRotation2d = Rotation2d.fromDegrees(pigeonYaw);
        m_TurretSubsystem.setVoltage(m_TurretSubsystem.closedLoopCalculate(pigeonYawRotation2d));
    
        lastVelocity = m_TurretSubsystem.getCurrentVelocity();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_TurretSubsystem.setVoltage(0.0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}