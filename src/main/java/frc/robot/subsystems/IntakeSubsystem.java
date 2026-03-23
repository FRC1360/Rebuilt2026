// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.ResetMode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.util.PIDLogger;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.controller.ProfiledPIDController;
import frc.robot.util.ClosedLoopConstants;

public class IntakeSubsystem extends SubsystemBase {

    private final ClosedLoopConstants defaultpivotProfiledPIDControllerConstants = new ClosedLoopConstants(
            0.4,
            0.0,
            0.0,
            270.0,
            360.0,
            0.15,
            0.017,
            0.002,
            0.0);

    private ProfiledPIDController pivotProfiledPIDController = new ProfiledPIDController(
            defaultpivotProfiledPIDControllerConstants.kP,
            defaultpivotProfiledPIDControllerConstants.kI,
            defaultpivotProfiledPIDControllerConstants.kD,
            new TrapezoidProfile.Constraints(
                    defaultpivotProfiledPIDControllerConstants.maxVelocity,
                    defaultpivotProfiledPIDControllerConstants.maxAcceleration));

    private ArmFeedforward pivotFeedForward = new ArmFeedforward(
            defaultpivotProfiledPIDControllerConstants.kG,
            defaultpivotProfiledPIDControllerConstants.kS,
            defaultpivotProfiledPIDControllerConstants.kV,
            defaultpivotProfiledPIDControllerConstants.kA);

    private final PIDLogger pidLogger = new PIDLogger(
            "Subsystems/" + getName(),
            defaultpivotProfiledPIDControllerConstants,
            constants -> {
                this.pivotProfiledPIDController.setPID(constants.kP, constants.kI, constants.kD);
                this.pivotProfiledPIDController.setConstraints(
                        new TrapezoidProfile.Constraints(constants.maxVelocity, constants.maxAcceleration));
                this.pivotFeedForward.setKa(constants.kA);
                this.pivotFeedForward.setKs(constants.kS);
                this.pivotFeedForward.setKv(constants.kV);
            });

    private final TalonFX rollerMotor = new TalonFX(IntakeConstants.ROLLER_VORTEX_CAN_ID);
    private final SparkFlex pivotMotor = new SparkFlex(IntakeConstants.PIVOT_VORTEX_CAN_ID, MotorType.kBrushless);

    private final FeedbackConfigs rollerFeedbackConfigs;
    private final MotorOutputConfigs rollerOutputConfigs;
    private final CurrentLimitsConfigs rollerCurrentLimitsConfigs;

    private SparkFlexConfig pivotConfig = new SparkFlexConfig();

    public Trigger IntakePivotAtTarget;

    // Variable used for FeedForward Calculation
    private double lastPivotVelocity = 0.0;

    public IntakeSubsystem() {

        rollerFeedbackConfigs = new FeedbackConfigs()
                .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor);

        rollerOutputConfigs = new MotorOutputConfigs()
                .withInverted(IntakeConstants.ROLLER_VORTEX_INVERTED)
                .withNeutralMode(NeutralModeValue.Coast);

        rollerCurrentLimitsConfigs = new CurrentLimitsConfigs()
                .withStatorCurrentLimit(IntakeConstants.ROLLER_VORTEX_STATOR_CURRENT_LIMIT)
                .withStatorCurrentLimitEnable(true);

        rollerMotor.getConfigurator().apply(rollerFeedbackConfigs);
        rollerMotor.getConfigurator().apply(rollerOutputConfigs);
        rollerMotor.getConfigurator().apply(rollerCurrentLimitsConfigs);

        pivotMotor.clearFaults();
        pivotConfig.inverted(IntakeConstants.PIVOT_VORTEX_INVERTED);
        pivotConfig.idleMode(IdleMode.kBrake);
        pivotConfig.smartCurrentLimit(
                IntakeConstants.PIVOT_VORTEX_STALL_CURRENT_LIMIT,
                IntakeConstants.PIVOT_VORTEX_FREE_CURRENT_LIMIT);
        pivotConfig.apply(
                new EncoderConfig()
                        .positionConversionFactor(IntakeConstants.PIVOT_POSITION_CONVERSION_RATIO)
                        .velocityConversionFactor(IntakeConstants.PIVOT_VELOCITY_CONVERSION_RATIO));
        pivotMotor.configure(
                pivotConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        pivotMotor.getEncoder().setPosition(IntakeConstants.PIVOT_STARTUP_ANGLE);

        pivotProfiledPIDController.setTolerance(Constants.IntakeConstants.PIVOT_PID_TOLERANCE);

        IntakePivotAtTarget = new Trigger(() -> (pivotProfiledPIDController.atGoal()));

    }

    public void setRollerSpeed(double speed) {
        rollerMotor.set(speed);
    }

    public void setRollerVoltage(double volts) {
        rollerMotor.setVoltage(volts);
    }

    public void setPivotSpeed(double speed) {
        pivotMotor.set(speed);
    }

    public void setPivotVoltage(double voltage) {
        pivotMotor.setVoltage(voltage);
    }

    public double getPivotVelocity() {
        return pivotMotor.getEncoder().getVelocity();
    }

    public double getPivotPosition() {
        return pivotMotor.getEncoder().getPosition();
    }

    public double closedLoopCalculate(double target) {
        double pidOutput = pivotProfiledPIDController.calculate(this.getPivotPosition(), target);
        double calculatedOutput = pidOutput + pivotFeedForward.calculateWithVelocities(
                Units.degreesToRadians(this.getPivotPosition()),
                lastPivotVelocity, pivotProfiledPIDController.getSetpoint().velocity);
        lastPivotVelocity = pivotProfiledPIDController.getSetpoint().velocity;
        return calculatedOutput;
    }

    public void resetPIDController() {
        pivotProfiledPIDController.reset(
                this.getPivotPosition(),
                this.getPivotVelocity());

        lastPivotVelocity = pivotProfiledPIDController.getSetpoint().velocity;
    }

    public void grabConstantsFromNetworkTables() {
        this.pidLogger.updateConstants();
    }

    @Override
    public void periodic() {
        pidLogger.logControllerOutputs(
                pivotProfiledPIDController.getGoal().position,
                pivotProfiledPIDController.getGoal().velocity,
                pivotProfiledPIDController.getSetpoint().position,
                pivotProfiledPIDController.getSetpoint().velocity,
                this.getPivotPosition(),
                this.getPivotVelocity(),
                pivotProfiledPIDController.getPositionError());
    }
}
