// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.util.ClosedLoopConstants;
import frc.robot.util.PIDLogger;

public class FlywheelSubsystem extends SubsystemBase {

    private final SparkFlex flywheelMotor = new SparkFlex(flywheelmotorID, MotorType.kBrushless);
    private final SparkFlex flywheelFollower = new SparkFlex(flywheelFollowerID, MotorType.kBrushless);

    private final SparkFlexConfig config = new SparkFlexConfig();
    private final SparkFlexConfig followerConfig = new SparkFlexConfig();

    private final RelativeEncoder flywheelEncoder;
    private final EncoderConfig motorEncoderConfig = new EncoderConfig();

    private static final int flywheelmotorID = 20;
    private static final int flywheelFollowerID = 21;

    private SysIdRoutine routine;

    public Trigger flywheelAtVelocity;

    private final ClosedLoopConstants defaultPIDConstants = new ClosedLoopConstants(
        0.03,
        0.0,
        0.0014,
        0.0,
        0.0,
        0.05901,
        0.10232,
        0.0,
        0.0
    );

    private PIDController flywheelPIDController = new PIDController(
        defaultPIDConstants.kP, defaultPIDConstants.kI, defaultPIDConstants.kD
    );
    private SimpleMotorFeedforward flywheelFFController = new SimpleMotorFeedforward(
        defaultPIDConstants.kS,
        defaultPIDConstants.kV,
        defaultPIDConstants.kA
    );

    private final PIDLogger pidLogger = new PIDLogger(
        "Subsystems/" + getName(),
        defaultPIDConstants,
        constants -> {
            this.flywheelPIDController.setPID(constants.kP, constants.kI, constants.kD);
            this.flywheelFFController.setKa(constants.kA);
            this.flywheelFFController.setKs(constants.kS);
            this.flywheelFFController.setKv(constants.kV);
        }
    );

    public FlywheelSubsystem() {

        motorEncoderConfig.velocityConversionFactor(Constants.FlywheelConstants.VELOCITY_CONVERSION_FACTOR);
        motorEncoderConfig.positionConversionFactor(Constants.FlywheelConstants.POSITION_CONVERSION_FACTOR);

        flywheelMotor.clearFaults();
        config.idleMode(IdleMode.kCoast);
        config.inverted(Constants.FlywheelConstants.FLYWHEEL_LEADER_INVERTED);
        config.smartCurrentLimit(
            Constants.FlywheelConstants.FLYWHEEL_STALL_CURRENT_LIMIT,
            Constants.FlywheelConstants.FLYWHEEL_FREE_CURRENT_LIMIT
        );
        config.apply(motorEncoderConfig);
        flywheelMotor.configure(
            config, 
            ResetMode.kResetSafeParameters, 
            PersistMode.kPersistParameters
        );

        flywheelEncoder = flywheelMotor.getEncoder();

        flywheelFollower.clearFaults();
        followerConfig.idleMode(IdleMode.kCoast);
        followerConfig.inverted(Constants.FlywheelConstants.FLYWHEEL_FOLLOWER_INVERTED);
        followerConfig.smartCurrentLimit(
            Constants.FlywheelConstants.FLYWHEEL_STALL_CURRENT_LIMIT,
            Constants.FlywheelConstants.FLYWHEEL_FREE_CURRENT_LIMIT
        );
        followerConfig.follow(flywheelMotor, true);
        flywheelFollower.configure(
            followerConfig, 
            ResetMode.kResetSafeParameters, 
            PersistMode.kPersistParameters
        );

        routine = new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(1).per(Second),
                Volts.of(7),
                Seconds.of(5)),
            new SysIdRoutine.Mechanism(
                (voltage) -> flywheelMotor.setVoltage(voltage),
                // Tell SysId how to record a frame of data for each motor on the mechanism
                // being
                log -> {
                    // Record a frame for the shooter motor.
                    log.motor("flywheel")
                            .voltage(Volts
                                    .of(flywheelMotor.getAppliedOutput() * RobotController.getBatteryVoltage()))
                            .linearPosition(Distance.ofBaseUnits(flywheelEncoder.getPosition(), Feet))
                            .linearVelocity(LinearVelocity.ofBaseUnits(flywheelEncoder.getVelocity(),
                                    Feet.per(Second)));
                },
                // Tell SysId to make generated commands require this subsystem, suffix test
                // state in
                // WPILog with this subsystem's name ("shooter")
                this
            )
        );

        flywheelPIDController.setTolerance(Constants.FlywheelConstants.PID_TOLERANCE);
        
        flywheelAtVelocity = new Trigger(() -> (flywheelPIDController.atSetpoint()));
    }

    public void resetPIDController() {
        flywheelPIDController.reset();
    }

    public double closedLoopCalculate(double target) {
        return flywheelPIDController.calculate(getFlywheelSpeed(), target)
                + flywheelFFController.calculate(target);
    }

    public void grabConstantsFromNetworkTables() {
        this.pidLogger.updateConstants();
    }

    public void setFlywheelSpeed(double speed) {
        flywheelMotor.set(speed);
    }

    public void setFlywheelVoltage(double voltage) {
        flywheelMotor.setVoltage(voltage);
    }

    public double getFlywheelSpeed() {
        return flywheelEncoder.getVelocity(); // surface velocity not motor v
    }

    public double getFlywheelPos() {
        return flywheelEncoder.getPosition();
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return routine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return routine.dynamic(direction);
    }

    @Override
    public void periodic() {
        pidLogger.logControllerOutputs(
                0.0,
                flywheelPIDController.getSetpoint(),
                0.0,
                flywheelPIDController.getSetpoint(),
                0.0,
                this.getFlywheelSpeed(),
                flywheelPIDController.getVelocityError());
    }
}