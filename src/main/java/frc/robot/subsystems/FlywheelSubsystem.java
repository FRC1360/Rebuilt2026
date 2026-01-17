// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FlywheelSubsystem extends SubsystemBase {
  private final SparkMax flywheelMotor;
  private final RelativeEncoder flywheelEncoder;
  
  private final PIDController flywheelPID;
  private final SimpleMotorFeedforward flywheelFF;
  
  private static final int flywheelmotorID = 0; //edit this yesyes
  private static final double kP = 0.0;
  private static final double kI = 0.0;
  private static final double kD = 0.0;
  private static final double kS = 0.0;
  private static final double kV = 0.0;
  private static final double kA = 0.0;
  
  public FlywheelSubsystem() {
    flywheelMotor = new SparkMax(flywheelmotorID, MotorType.kBrushless);
    
    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(IdleMode.kCoast);
    config.smartCurrentLimit(40);
    
    flywheelMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
    flywheelEncoder = flywheelMotor.getEncoder();
    
    flywheelPID = new PIDController(kP, kI, kD);
    
    flywheelFF = new SimpleMotorFeedforward(kS, kV, kA);
  }

  public void setFlywheelSpeed(double speed) {
    flywheelMotor.set(speed);
  }
  
  public double getFlywheelSpeed() {
    return flywheelEncoder.getVelocity();
  }

  public PIDController getFlywheelPID() {
    return flywheelPID;
  }

  public SimpleMotorFeedforward getFlywheelFF() {
    return flywheelFF;
  }

  @Override
  public void periodic() {}
}
