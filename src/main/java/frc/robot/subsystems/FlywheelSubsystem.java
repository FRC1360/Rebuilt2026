// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.util.ClosedLoopConstants;
import frc.robot.util.PIDLogger;

public class FlywheelSubsystem extends SubsystemBase {

  private ProfiledPIDController flywheelPIDController = new ProfiledPIDController(
        0.0, 0.0, 0.0,
        new TrapezoidProfile.Constraints(0.0, 0.0)
    );
  private SimpleMotorFeedforward flywheelFFController = new SimpleMotorFeedforward(0.0, 0.0, 0.0);

  private final ClosedLoopConstants defaultPIDConstants = new ClosedLoopConstants(
    0.05, 0.0, 0.003, 0.0, 0.0, 0.05901, 0.10232, 0.0, 0.0 
  );

  private final PIDLogger pidLogger = new PIDLogger(
    "Subsystems" + getName(), 
    defaultPIDConstants, 
    constants -> {
        this.flywheelPIDController.setPID(constants.kP, constants.kI, constants.kD);
        this.flywheelPIDController.setConstraints(
            new TrapezoidProfile.Constraints(constants.maxVelocity, constants.maxAcceleration)
          );
        this.flywheelFFController.setKa(constants.kA);
        this.flywheelFFController.setKs(constants.kS);
        this.flywheelFFController.setKv(constants.kV);
    }
  );

  private final SparkFlex flywheelMotor;
  private final SparkFlex flywheelFollower;
  private final RelativeEncoder flywheelEncoder;
  private final EncoderConfig motorEncoderConfig;
  private final SparkFlexConfig config;
  private final SparkFlexConfig followerConfig;

  private static final int flywheelmotorID = 20; 
  private static final int flywheelFollowerID = 21;

  private SysIdRoutine routine;

  public FlywheelSubsystem() {

    config = new SparkFlexConfig();
    followerConfig = new SparkFlexConfig();

    flywheelMotor = new SparkFlex(flywheelmotorID, MotorType.kBrushless);
    flywheelFollower = new SparkFlex(flywheelFollowerID, MotorType.kBrushless);
    motorEncoderConfig = new EncoderConfig();


    motorEncoderConfig.velocityConversionFactor((4 * Math.PI) / (60 * 12));
    motorEncoderConfig.positionConversionFactor((4 * Math.PI) / (12));

    config.idleMode(IdleMode.kCoast);
    // config.smartCurrentLimit(40);
    config.apply(motorEncoderConfig);

    followerConfig.idleMode(IdleMode.kCoast);
    followerConfig.follow(flywheelMotor,true);

    flywheelMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    flywheelFollower.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    flywheelEncoder = flywheelMotor.getEncoder();

    routine = new SysIdRoutine(
      new SysIdRoutine.Config(
        Volts.of(1).per(Second),
        Volts.of(7),
        Seconds.of(5)
      ),
      new SysIdRoutine.Mechanism(
        (voltage) -> flywheelMotor.setVoltage(voltage),
        // Tell SysId how to record a frame of data for each motor on the mechanism being
        log -> {
          // Record a frame for the shooter motor.
          log.motor("flywheel")
              .voltage(Volts.of(flywheelMotor.getAppliedOutput() * RobotController.getBatteryVoltage()))
              .linearPosition(Distance.ofBaseUnits(flywheelEncoder.getPosition(), Feet))
              .linearVelocity(LinearVelocity.ofBaseUnits(flywheelEncoder.getVelocity(), Feet.per(Second)));
        },
        // Tell SysId to make generated commands require this subsystem, suffix test state in
        // WPILog with this subsystem's name ("shooter")
        this));
  }

  public double closedLoopCalculate(double target) {
    flywheelPIDController.setGoal(target);
    return 
        flywheelPIDController.calculate(getFlywheelSpeed()) 
        + flywheelFFController.calculate(flywheelPIDController.getSetpoint().velocity); // double check pid input
  }

  public void setFlywheelSpeed(double speed) {
    flywheelMotor.set(speed);
  }

  public void setFlywheelVoltage(double voltage) {
    flywheelMotor.setVoltage(voltage);
  }

  public double getFlywheelSpeed() {
    return flywheelEncoder.getVelocity(); //surface velocity not motor v
  }

  public double getFlywheelPos() {
    return flywheelEncoder.getPosition(); 
  }

  @Override
  public void periodic() {
    pidLogger.updateConstants();
    pidLogger.logControllerOutputs(
        0.0,
        flywheelPIDController.getSetpoint().velocity,
        0.0,
        this.getFlywheelSpeed()
    );
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return routine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) { 
    return routine.dynamic(direction);
  }


}









