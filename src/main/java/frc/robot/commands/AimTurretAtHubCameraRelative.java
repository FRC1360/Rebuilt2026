// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;

import org.photonvision.PhotonUtils;

import edu.wpi.first.math.MathUtil;
import frc.robot.subsystems.OrbitCamera;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.util.FieldConstants;



/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AimTurretAtHubCameraRelative extends Command {

    private final double default_kP = 0.055;
    private final double default_kI = 0.01;
    private final double default_kD = 0.0;
    private final double default_maxVelocity = 3000;
    private final double default_maxAcceleration = 3500;
    private final double default_kS = 0.11;
    private final double default_kV = 0.0022;
    private final double default_kA = 0.0;

    private final NetworkTable loggingTable;
    private final DoublePublisher controlLoopOutputPublisher;
    private final DoublePublisher setpointPublisher;
    private final DoublePublisher setpointVelocityPublisher;
    private final DoublePublisher currentAnglePublisher;
    private final DoublePublisher currentVelocityPublisher;
    private final DoublePublisher deltaYawPublisher;
    

    private final DoubleEntry kP_Entry;
    private final DoubleEntry kI_Entry;
    private final DoubleEntry kD_Entry;
    private final DoubleEntry maxAcceleration_Entry;
    private final DoubleEntry maxVelocity_Entry;
    private final DoubleEntry kS_Entry;
    private final DoubleEntry kV_Entry;
    private final DoubleEntry kA_Entry;


    private final TurretSubsystem m_TurretSubsystem;
    private final ProfiledPIDController m_pidController;
    private final SimpleMotorFeedforward m_feedForward;

    private double lastVelocity;
    private Pose2d currentTurretPose;
    private double pidControllerOutput;
    private double feedForwardControllerOutput;
    private double wrappedTarget;

    /** Creates a new SetHoodAngleCommand. */
    public AimTurretAtHubCameraRelative(TurretSubsystem turretSubsystem) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.m_TurretSubsystem = turretSubsystem;

        this.m_pidController = new ProfiledPIDController(
            default_kP,
            default_kI,
            default_kD,
            new TrapezoidProfile.Constraints(default_maxVelocity, default_maxAcceleration)
        );
        this.m_feedForward = new SimpleMotorFeedforward(
            default_kS,
            default_kV,
            default_kA
        );

        loggingTable = NetworkTableInstance.getDefault().getTable("Commands/"+getName());
        controlLoopOutputPublisher = loggingTable.getDoubleTopic("PID Output").publish();
        setpointPublisher = loggingTable.getDoubleTopic("Setpoint Degrees").publish();
        currentAnglePublisher = loggingTable.getDoubleTopic("Current Degrees").publish();
        currentVelocityPublisher = loggingTable.getDoubleTopic("Current Velocity Degrees per Sec").publish();
        setpointVelocityPublisher = loggingTable.getDoubleTopic("Setpoint Velocity Degrees per Sec").publish();
        deltaYawPublisher = loggingTable.getDoubleTopic("Photon Utils Yaw Delta Degrees").publish();

        kP_Entry = loggingTable.getDoubleTopic("kP").getEntry(default_kP);
        kI_Entry = loggingTable.getDoubleTopic("kI").getEntry(default_kI);
        kD_Entry = loggingTable.getDoubleTopic("kD").getEntry(default_kD);
        maxVelocity_Entry = loggingTable.getDoubleTopic("Max Velocity").getEntry(default_maxVelocity);
        maxAcceleration_Entry = loggingTable.getDoubleTopic("Max Acceleration").getEntry(default_maxAcceleration);
        kS_Entry = loggingTable.getDoubleTopic("kS").getEntry(default_kS);
        kV_Entry = loggingTable.getDoubleTopic("kV").getEntry(default_kV);
        kA_Entry = loggingTable.getDoubleTopic("kA").getEntry(default_kA);
     

        kP_Entry.set(default_kP);
        kI_Entry.set(default_kI);
        kD_Entry.set(default_kD);
        maxVelocity_Entry.set(default_maxVelocity);
        maxAcceleration_Entry.set(default_maxAcceleration);
        kS_Entry.set(default_kS);
        kV_Entry.set(default_kV);
        kA_Entry.set(default_kA);


        this.pidControllerOutput = 0.0;
        this.feedForwardControllerOutput = 0.0;

        addRequirements(m_TurretSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        
        m_pidController.setP(kP_Entry.get());
        m_pidController.setI(kI_Entry.get());
        m_pidController.setD(kD_Entry.get());
        m_pidController.setConstraints(
            new TrapezoidProfile.Constraints(
                maxVelocity_Entry.get(), 
                maxAcceleration_Entry.get()
            )
        );
        m_pidController.reset(
            m_TurretSubsystem.getCurrentAngle(),
            m_TurretSubsystem.getCurrentVelocity()
        );

        m_feedForward.setKs(kS_Entry.get());
        m_feedForward.setKv(kV_Entry.get());
        m_feedForward.setKa(kA_Entry.get());

        lastVelocity = m_TurretSubsystem.getCurrentVelocity();
    }

    private double calculateWrapAround(){
        currentTurretPose = m_TurretSubsystem.getEstimatedPose();
        
        double currentAngle = 0.0;
        Rotation2d targetAngleRotation = PhotonUtils.getYawToPose(currentTurretPose, FieldConstants.redAllianceHubPose).times(-1);
        deltaYawPublisher.accept(PhotonUtils.getYawToPose(currentTurretPose, FieldConstants.redAllianceHubPose).getDegrees());
        double wrappedTargetValue = MathUtil.inputModulus(targetAngleRotation.getDegrees(), -180, 180);
        wrappedTarget = wrappedTargetValue;

        double negativePath;
        double positivePath;

        if (wrappedTargetValue >= 0){
            positivePath = wrappedTargetValue;
            negativePath = wrappedTargetValue - 360;
        }
        else{
            positivePath =  wrappedTargetValue + 360; 
            negativePath = wrappedTargetValue;

        }

        if (currentAngle > 0 && negativePath > -160.0){
            return negativePath;
        }

        else if (currentAngle < 0 && positivePath < 160){
            return positivePath;
        }

        if(Math.abs(currentAngle - positivePath) > Math.abs(currentAngle - negativePath)){
            return negativePath;
        }
        return positivePath;


    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        wrappedTarget = calculateWrapAround();
        m_pidController.setGoal(wrappedTarget);
        
        pidControllerOutput = m_pidController.calculate(m_TurretSubsystem.getCurrentAngle());
        feedForwardControllerOutput = m_feedForward.calculateWithVelocities(
            lastVelocity,
            m_pidController.getSetpoint().velocity
        );

        controlLoopOutputPublisher.set(feedForwardControllerOutput);
        setpointPublisher.set(wrappedTarget);
        setpointVelocityPublisher.set(m_pidController.getSetpoint().velocity);
        currentAnglePublisher.set(m_TurretSubsystem.getCurrentAngle());
        currentVelocityPublisher.set(m_TurretSubsystem.getCurrentVelocity());

        m_TurretSubsystem.setVoltage(pidControllerOutput + feedForwardControllerOutput);

        lastVelocity = m_TurretSubsystem.getCurrentVelocity();
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