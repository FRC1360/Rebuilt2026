// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.util.ClosedLoopConstants;
import frc.robot.util.PIDLogger;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class TurretSubsystem extends SubsystemBase {

    private final ClosedLoopConstants defaultPIDConstants = new ClosedLoopConstants(
        0.055,
        0.01,
        0.0,
        3000,
        3500,
        0.11,
        0.0022,
        0.0,
        0
    );
    private final double hardstopAngleDegrees = 0.0;
    private final Pose2d robotToturret = new Pose2d(
        new Translation2d(0.225, 0.0),
        new Rotation2d()
    );

    private final ProfiledPIDController m_pidController = new ProfiledPIDController(
        defaultPIDConstants.kP, defaultPIDConstants.kI, defaultPIDConstants.kD,
        new TrapezoidProfile.Constraints(defaultPIDConstants.maxVelocity, defaultPIDConstants.maxAcceleration)
    );
    private final SimpleMotorFeedforward m_feedForward = new SimpleMotorFeedforward(
        defaultPIDConstants.kS,
        defaultPIDConstants.kV,
        defaultPIDConstants.kA
    );

    private final PIDLogger pidLogger = new PIDLogger(
        "Subsystems/" + getName(),
        defaultPIDConstants,
        constants -> {
            this.m_pidController.setPID(constants.kP, constants.kI, constants.kD);
            this.m_pidController.setConstraints(
                new TrapezoidProfile.Constraints(constants.maxVelocity, constants.maxAcceleration)
            );
            this.m_feedForward.setKa(constants.kA);
            this.m_feedForward.setKs(constants.kS);
            this.m_feedForward.setKv(constants.kV);
        }
    );

    private final SparkMax motor;
    private final EncoderConfig encoderConfig;
    private final SparkMaxConfig motorConfig;
    private static final int TurretMotorID = 60;
    private static final double gearRatio = (1.0 / 7.0) * 360.0;
    private final SysIdRoutine routine;

    private double pidControllerOutput;
    private double feedForwardControllerOutput;
    private double wrappedTarget;

    private final OrbitCamera orbitCamera;
    private Pose2d estimatedPose;

    public TurretSubsystem() {
        motor = new SparkMax(TurretMotorID, MotorType.kBrushless);
        motorConfig = new SparkMaxConfig();
        encoderConfig = new EncoderConfig();

        encoderConfig.velocityConversionFactor(gearRatio / 60.0); // per minute to per second
        encoderConfig.positionConversionFactor(gearRatio);
        motorConfig.idleMode(IdleMode.kBrake);
        motorConfig.inverted(true);
        motorConfig.apply(encoderConfig);

        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        motor.getEncoder().setPosition(hardstopAngleDegrees);

        this.pidControllerOutput = 0.0;
        this.feedForwardControllerOutput = 0.0;

        routine = new SysIdRoutine(
                new SysIdRoutine.Config(
                        Volts.of(0.5).per(Second),
                        Volts.of(3.5),
                        Seconds.of(10)),
                new SysIdRoutine.Mechanism(
                        (voltage) -> motor.setVoltage(voltage),
                        // Tell SysId how to record a frame of data for each motor on the mechanism
                        // being
                        log -> {
                            // Record a frame for the turret motor.
                            log.motor("turret")
                                    .voltage(Volts.of(motor.getAppliedOutput() * RobotController.getBatteryVoltage()))
                                    .angularPosition(Angle.ofBaseUnits(motor.getEncoder().getPosition(), Degrees))
                                    .angularVelocity(AngularVelocity.ofBaseUnits(motor.getEncoder().getVelocity(),
                                            Degrees.per(Second)));
                        },
                        // Tell SysId to make generated commands require this subsystem, suffix test
                        // state in
                        // WPILog with this subsystem's name ("turret")
                        this));

        orbitCamera = new OrbitCamera(
                new Transform3d(
                        new Translation3d(0.05, 0, 0.045),
                        new Rotation3d(0, -Math.toRadians(10), Math.toRadians(0))),
                "photoncamera_turret");
    }

    public void grabConstantsFromNetworkTables() {
        this.pidLogger.updateConstants();
    }

    public void setVoltage(double volts) {
        motor.setVoltage(volts);
    }

    public double getCurrentVelocity() {
        return motor.getEncoder().getVelocity();
    }

    public double getCurrentAngle() {
        return motor.getEncoder().getPosition();
    }

    public Rotation2d getCurrentRotation() {
        return Rotation2d.fromDegrees(this.getCurrentAngle()).plus(this.robotToturret.getRotation());
    }

    private double calculateWrapAround(Rotation2d target) {

        double currentAngle = this.getCurrentAngle();
        double wrappedTargetValue = MathUtil.inputModulus(target.getDegrees(), -180, 180);
        double negativePath, positivePath;

        if (wrappedTargetValue >= 0) {
            positivePath = wrappedTargetValue;
            negativePath = wrappedTargetValue - 360;
        } else {
            positivePath = wrappedTargetValue + 360;
            negativePath = wrappedTargetValue;
        }

        if (currentAngle > 0 && negativePath > -160.0) {
            return negativePath;
        } else if (currentAngle < 0 && positivePath < 160) {
            return positivePath;
        }

        if (Math.abs(currentAngle - positivePath) > Math.abs(currentAngle - negativePath)) {
            return negativePath;
        }
        return positivePath;

    }

    public double closedLoopCalculate(Rotation2d target) {
        wrappedTarget = this.calculateWrapAround(target.minus(this.robotToturret.getRotation()));

        m_pidController.setGoal(wrappedTarget);
        pidControllerOutput = m_pidController.calculate(this.getCurrentAngle());
        feedForwardControllerOutput = m_feedForward.calculate(m_pidController.getSetpoint().velocity);

        return pidControllerOutput + feedForwardControllerOutput;
    }

    @Override
    public void periodic() {
        pidLogger.logControllerOutputs(
            m_pidController.getGoal().position,
            m_pidController.getGoal().velocity,
            m_pidController.getSetpoint().position,
            m_pidController.getSetpoint().velocity,
            this.getCurrentAngle(),
            this.getCurrentVelocity()
        );

        this.updatePose();
    }

    private void updatePose() {
        orbitCamera.updatePipelineResults();
        Optional<EstimatedRobotPose> visionEst = Optional.empty();
        for (var result : orbitCamera.getPipelineResults()) {
            visionEst = orbitCamera.getPhotonPoseEstimator().estimateCoprocMultiTagPose(result);
            if (visionEst.isEmpty()) {
                visionEst = orbitCamera.getPhotonPoseEstimator().estimateLowestAmbiguityPose(result);
            }
            orbitCamera.updateEstimationStdDevs(visionEst, result.getTargets());

            visionEst.ifPresent(
                    est -> {
                        // Change our trust in the measurement based on the tags we can see
                        // var estStdDevs = orbitCamera.getEstimationStdDevs();

                        orbitCamera.updateStructPublisher(est.estimatedPose.toPose2d());
                        this.estimatedPose = est.estimatedPose.toPose2d();
                    });
        }
    }

    public Pose2d getEstimatedPose() {
        return this.estimatedPose;
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return routine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return routine.dynamic(direction);
    }
}