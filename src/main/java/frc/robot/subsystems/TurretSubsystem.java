// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.TurretConstants;
import frc.robot.util.ClosedLoopConstants;
import frc.robot.util.PIDLogger;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import com.ctre.phoenix6.controls.VoltageOut;


import java.util.Optional;
import java.util.function.BooleanSupplier;

import org.photonvision.EstimatedRobotPose;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;


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
    private final VoltageOut turretVoltageRequest = new VoltageOut(0.0);

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

    private final TalonFX motor = new TalonFX(TurretConstants.KRAKEN_CAN_ID);

    private double pidControllerOutput;
    private double feedForwardControllerOutput;
    private double wrappedTarget;

    private final OrbitCamera orbitCamera;
    private Pose2d photonCameraEstimatedPose;
    private double photonCameraEstimatedPoseTimestamp;

    private final FeedbackConfigs motorFeedbackConfigs;
    private final MotorOutputConfigs motorOutputConfigs;
    private final CurrentLimitsConfigs motorCurrentLimitsConfigs;

    public TurretSubsystem() {
        motorFeedbackConfigs = new FeedbackConfigs()
            .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
            .withSensorToMechanismRatio(TurretConstants.MECHANISM_CONVERSION_FACTOR);

        motorOutputConfigs = new MotorOutputConfigs()
            .withInverted(TurretConstants.KRAKEN_INVERTED_VALUE)
            .withNeutralMode(NeutralModeValue.Brake);
        
        motorCurrentLimitsConfigs = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(TurretConstants.KRAKEN_STATOR_CURRENT_LIMIT)
            .withStatorCurrentLimitEnable(true);

        motor.getConfigurator().apply(motorFeedbackConfigs);
        motor.getConfigurator().apply(motorOutputConfigs);
        motor.getConfigurator().apply(motorCurrentLimitsConfigs);
        motor.setPosition(TurretConstants.ENCODER_STARTUP_ANGLE_DEGREES);

        this.pidControllerOutput = 0.0;
        this.feedForwardControllerOutput = 0.0;

        orbitCamera = new OrbitCamera(TurretConstants.TURRET_CENTER_TO_CAMERA, "photoncamera_turret");
    }

    public void grabConstantsFromNetworkTables() {
        this.pidLogger.updateConstants();
    }

    public void resetPIDController() {
        m_pidController.reset(
            this.getCurrentEncoderAngle(),
            this.getCurrentEncoderVelocity()
        );
    }

    public void setVoltage(double volts) {
        motor.setVoltage(volts);
    }

    public double getCurrentEncoderVelocity() {
        return motor.getVelocity().getValueAsDouble();
    }

    public double getCurrentEncoderAngle() {
        return motor.getPosition().getValueAsDouble();
    }

    public Rotation2d getCurrentRobotRelativeRotation() {
        return Rotation2d.fromDegrees(this.getCurrentEncoderAngle()).plus(TurretConstants.ROBOT_TO_TURRET_CENTER.getRotation());
    }

    private double calculateWrapAround(Rotation2d target) {
        //Converts the angle between the range of 0 to 360 to -180 to 180
        double currentAngle = this.getCurrentEncoderAngle();
        double wrappedTargetValue = MathUtil.inputModulus(target.getDegrees(), -180, 180);
        double negativePath, positivePath;

        //Determines the co-terminal angles which are the negative and positive paths that the turret will take.
        if (wrappedTargetValue >= 0) {
            positivePath = wrappedTargetValue;
            negativePath = wrappedTargetValue - 360;
        } else {
            positivePath = wrappedTargetValue + 360;
            negativePath = wrappedTargetValue;
        }

        //If the turret is crossing the positive threshold from the positive direction, take the negative path.
        if (currentAngle > 0 && negativePath > TurretConstants.NEGATIVE_THRESHOLD) {
            return negativePath;
        
        //If the turret is crossing the negative threshold from the negative direction, take the positive path.
        } else if (currentAngle < 0 && positivePath < TurretConstants.POSITIVE_THRESHOLD) {
            return positivePath;
        }

        //Determines and returns the shorter path by magnitude with respect to the current position.
        if (Math.abs(currentAngle - positivePath) > Math.abs(currentAngle - negativePath)) {
            return negativePath;
        }
        return positivePath;
    }

    public double closedLoopCalculate(Rotation2d target) {
        wrappedTarget = this.calculateWrapAround(target.minus(TurretConstants.ROBOT_TO_TURRET_CENTER.getRotation()));

        m_pidController.setGoal(wrappedTarget);
        pidControllerOutput = m_pidController.calculate(this.getCurrentEncoderAngle());
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
            this.getCurrentEncoderAngle(),
            this.getCurrentEncoderVelocity(),
            m_pidController.getPositionError()
        );

        this.updatePose();
    }

    private void updatePose() {
        orbitCamera.updatePipelineResults();
        //Creates an optional object to prevent NullPointerException if the pipeline results return nothing.
        Optional<EstimatedRobotPose> visionEst = Optional.empty();
        //
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

                        this.photonCameraEstimatedPose = est.estimatedPose.toPose2d();
                        this.photonCameraEstimatedPoseTimestamp = est.timestampSeconds;
                        orbitCamera.updateStructPublisher(this.photonCameraEstimatedPose);
                    });
        }
    }

    public Pose2d getPhotonCameraEstimatedPose() {
        return this.photonCameraEstimatedPose;
    }
    public double getPhotonCameraEstimatedPoseTimestamp() {
        return this.photonCameraEstimatedPoseTimestamp;
    }

    public Trigger pidAtGoal() {
        BooleanSupplier supplier = () -> m_pidController.atGoal(); 
        return new Trigger(supplier);
    }

     private SysIdRoutine sysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(
            Volts.of(0.1).per(Second),  // Ramp Rate of 0.1V/s
            Volts.of(0.4),              // Dynamic Step Voltage of 0.4V
            Seconds.of(10),                         // Use default timeout (10 s)
            state -> SignalLogger.writeString("Turret_SysID_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
         (volts) -> motor.setControl(turretVoltageRequest.withOutput(volts.in(Volts))),
         null,
         this
      )
    );

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
       return sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
       return sysIdRoutine.dynamic(direction);
    }

}
