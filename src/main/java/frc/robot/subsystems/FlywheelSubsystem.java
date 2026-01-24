// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import org.littletonrobotics.urcl.URCL;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class FlywheelSubsystem extends SubsystemBase {
  private final SparkFlex flywheelMotor;
  private final RelativeEncoder flywheelEncoder;
  private final EncoderConfig motorEncoderConfig;
  private final SparkFlexConfig config;


  private final PIDController flywheelPID;
  private final SimpleMotorFeedforward flywheelFF;

  private static final int flywheelmotorID = 0; //edit this yesyesyes
  private static final double kP = 0.0;
  private static final double kI = 0.0;
  private static final double kD = 0.0;
  private static final double kS = 0.0;
  private static final double kV = 0.0;
  private static final double kA = 0.0;

  private SysIdRoutine routine;

  public FlywheelSubsystem() {
    
    config = new SparkFlexConfig();
    flywheelPID = new PIDController(kP, kI, kD);
    flywheelFF = new SimpleMotorFeedforward(kS, kV, kA);
    flywheelMotor = new SparkFlex(flywheelmotorID, MotorType.kBrushless);
    motorEncoderConfig = new EncoderConfig();

    motorEncoderConfig.velocityConversionFactor(4 * Math.PI);

    config.idleMode(IdleMode.kCoast);
    config.smartCurrentLimit(20);
    config.apply(motorEncoderConfig);

    flywheelMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    flywheelEncoder = flywheelMotor.getEncoder();

    DataLogManager.start();
    URCL.start();

    

    routine = new SysIdRoutine(
      new SysIdRoutine.Config(),
      new SysIdRoutine.Mechanism(
        (voltage) -> flywheelMotor.setVoltage(voltage),
        null,
        this
      )
    );
  }

  public void setFlywheelSpeed(double speed) {
    flywheelMotor.set(speed);
  }

  public double getFlywheelSpeed() {
    return flywheelEncoder.getVelocity(); //surface velocity not motor v
  }

  public PIDController getFlywheelPID() {
    return flywheelPID;
  }

  public SimpleMotorFeedforward getFlywheelFF() {
    return flywheelFF;
  }

  @Override
  public void periodic() {
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return routine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) { 
    return routine.dynamic(direction);
  }

}









