// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.util.PIDLogger;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.controller.ProfiledPIDController;
import frc.robot.util.ClosedLoopConstants;

public class IntakeSubsystem extends SubsystemBase {

    private final ClosedLoopConstants defaultPivotPIDConstants = new ClosedLoopConstants(
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0);

    private ProfiledPIDController pivotPID = new ProfiledPIDController(
            defaultPivotPIDConstants.kP,
            defaultPivotPIDConstants.kI,
            defaultPivotPIDConstants.kD,
            new TrapezoidProfile.Constraints(
                    defaultPivotPIDConstants.maxVelocity,
                    defaultPivotPIDConstants.maxAcceleration));

    private ArmFeedforward pivotFeedForward = new ArmFeedforward(
            defaultPivotPIDConstants.kG,
            defaultPivotPIDConstants.kS,
            defaultPivotPIDConstants.kV,
            defaultPivotPIDConstants.kA);

    private final PIDLogger pidLogger = new PIDLogger(
            "Subsystems/" + getName(),
            defaultPivotPIDConstants,
            constants -> {
                this.pivotPID.setPID(constants.kP, constants.kI, constants.kD);
                this.pivotPID.setConstraints(
                        new TrapezoidProfile.Constraints(constants.maxVelocity, constants.maxAcceleration));
                this.pivotFeedForward.setKa(constants.kA);
                this.pivotFeedForward.setKs(constants.kS);
                this.pivotFeedForward.setKv(constants.kV);
            });

    private final SparkFlex rollerMotor = new SparkFlex(IntakeConstants.ROLLER_VORTEX_CAN_ID, MotorType.kBrushless);
    private final SparkFlex pivotMotor = new SparkFlex(IntakeConstants.PIVOT_VORTEX_CAN_ID, MotorType.kBrushless);

    private SparkFlexConfig rollerConfig = new SparkFlexConfig();
    private SparkFlexConfig pivotConfig = new SparkFlexConfig();

    // Variable used for FeedForward Calculation
    private double lastPivotVelocity = 0.0;

    public IntakeSubsystem() {
        rollerMotor.clearFaults();
        rollerConfig.inverted(IntakeConstants.ROLLER_VORTEX_INVERTED);
        rollerConfig.idleMode(IdleMode.kCoast);
        rollerConfig.smartCurrentLimit(
                IntakeConstants.ROLLER_VORTEX_STALL_CURRENT_LIMIT,
                IntakeConstants.ROLLER_VORTEX_FREE_CURRENT_LIMIT);
        rollerMotor.configure(
                rollerConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

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
        double pidOutput = pivotPID.calculate(this.getPivotPosition(), target);
        double calculatedOutput = pidOutput + pivotFeedForward.calculateWithVelocities(
                Units.degreesToRadians(this.getPivotPosition()),
                lastPivotVelocity, pivotPID.getSetpoint().velocity);
        lastPivotVelocity = pivotPID.getSetpoint().velocity;
        return calculatedOutput;
    }

    public void resetPIDController() {
        pivotPID.reset(
                this.getPivotPosition(),
                this.getPivotVelocity());

        lastPivotVelocity = pivotPID.getSetpoint().velocity;
    }

    public void grabConstantsFromNetworkTables() {
        this.pidLogger.updateConstants();
    }

    @Override
    public void periodic() {
        pidLogger.logControllerOutputs(
                pivotPID.getGoal().position,
                pivotPID.getGoal().velocity,
                pivotPID.getSetpoint().position,
                pivotPID.getSetpoint().velocity,
                this.getPivotPosition(),
                this.getPivotPosition(),
                pivotPID.getPositionError());
    }
}
