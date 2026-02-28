// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.HoodConstants;
import frc.robot.util.ClosedLoopConstants;
import frc.robot.util.PIDLogger;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class HoodSubsystem extends SubsystemBase {

    private final ClosedLoopConstants defaultPIDConstants = new ClosedLoopConstants(
            0.06,
            0.07,
            0.0,
            90.0,
            360.0,
            0.13988,
            0.0092414,
            0.0023902,
            0.0);

    private ProfiledPIDController hoodProfiledPIDController = new ProfiledPIDController(
            defaultPIDConstants.kP, defaultPIDConstants.kI, defaultPIDConstants.kD,
            new TrapezoidProfile.Constraints(defaultPIDConstants.maxVelocity, defaultPIDConstants.maxAcceleration));
    private SimpleMotorFeedforward hoodFFController = new SimpleMotorFeedforward(
            defaultPIDConstants.kS,
            defaultPIDConstants.kV,
            defaultPIDConstants.kA);

    private final PIDLogger pidLogger = new PIDLogger(
            "Subsystems/" + getName(),
            defaultPIDConstants,
            constants -> {
                this.hoodProfiledPIDController.setPID(constants.kP, constants.kI, constants.kD);
                this.hoodProfiledPIDController.setConstraints(
                        new TrapezoidProfile.Constraints(constants.maxVelocity, constants.maxAcceleration));
                this.hoodFFController.setKa(constants.kA);
                this.hoodFFController.setKs(constants.kS);
                this.hoodFFController.setKv(constants.kV);
            });

    private final TalonFX hoodMotor = new TalonFX(HoodConstants.KRAKEN_CAN_ID);
    private final VoltageOut hoodVoltageRequest = new VoltageOut(0.0);

    private final FeedbackConfigs motorFeedbackConfigs;
    private final MotorOutputConfigs motorOutputConfigs;
    private final CurrentLimitsConfigs motorCurrentLimitsConfigs;

    public Trigger hoodAtTarget;

    /** Creates a new HoodSubsystem. */
    public HoodSubsystem() {
        motorFeedbackConfigs = new FeedbackConfigs()
                .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                .withSensorToMechanismRatio(HoodConstants.MECHANISM_CONVERSION_FACTOR);

        motorOutputConfigs = new MotorOutputConfigs()
                .withInverted(HoodConstants.KRAKEN_INVERTED_VALUE)
                .withNeutralMode(NeutralModeValue.Brake);

        motorCurrentLimitsConfigs = new CurrentLimitsConfigs()
                .withStatorCurrentLimit(HoodConstants.KRAKEN_STATOR_CURRENT_LIMIT)
                .withStatorCurrentLimitEnable(true);

        hoodMotor.getConfigurator().apply(motorFeedbackConfigs);
        hoodMotor.getConfigurator().apply(motorOutputConfigs);
        hoodMotor.getConfigurator().apply(motorCurrentLimitsConfigs);
        hoodMotor.setPosition(HoodConstants.STARTUP_ANGLE_DEGREES);

        hoodProfiledPIDController.setTolerance(Constants.HoodConstants.PID_TOLERANCE);

        hoodAtTarget = new Trigger(() -> (hoodProfiledPIDController.atGoal()));
    }

    @Override
    public void periodic() {
        pidLogger.logControllerOutputs(
                hoodProfiledPIDController.getGoal().position,
                hoodProfiledPIDController.getGoal().velocity,
                hoodProfiledPIDController.getSetpoint().position,
                hoodProfiledPIDController.getSetpoint().velocity,
                this.getCurrentAngle(),
                this.getCurrentVelocity(),
                hoodProfiledPIDController.getPositionError());
    }

    public void setHoodMotorVoltage(double volts) {
        hoodMotor.setVoltage(volts);
    }

    public double getCurrentAngle() {
        return hoodMotor.getPosition().getValueAsDouble();
    }

    public double getCurrentVelocity() {
        return hoodMotor.getVelocity().getValueAsDouble();
    }

    public double closedLoopCalculate(double target) {
        return hoodProfiledPIDController.calculate(getCurrentAngle(), target)
                + hoodFFController.calculate(hoodProfiledPIDController.getSetpoint().velocity); // double check pid input
    }

    public void resetPIDController() {
        hoodProfiledPIDController.reset(
                this.getCurrentAngle(),
                this.getCurrentVelocity());
    }

    public void grabConstantsFromNetworkTables() {
        this.pidLogger.updateConstants();
    }

    private SysIdRoutine sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                    Volts.of(0.1).per(Second), // Ramp Rate of 0.1V/s
                    Volts.of(0.4), // Dynamic Step Voltage of 0.4V
                    Seconds.of(10), // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("Hood_SysID_State", state.toString())
            // tHood angle, aHood angle, tFlywheel speed, aFlywheel speed
            ),
            new SysIdRoutine.Mechanism(
                    (volts) -> hoodMotor.setControl(hoodVoltageRequest.withOutput(volts.in(Volts))),
                    null,
                    this));

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }
}
