// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IndexSubsystem extends SubsystemBase {
  /** Creates a new IndexSubsystem. */

  private SparkFlex rightMotor;
  private SparkFlex leftMotor;
  private SparkFlex kickupMotor;
  private int rightMotorID = 0;
  private int leftMotorID = 0;
  private int kickupID = 0;
  private SparkFlexConfig leftConfig;
  private SparkFlexConfig rightConfig;
  private SparkFlexConfig kickupConfig;
  private boolean inverted;
  private double kickupSpeed = 0.3;

  public IndexSubsystem() {
    rightMotor = new SparkFlex(rightMotorID, MotorType.kBrushless);
    leftMotor = new SparkFlex(leftMotorID, MotorType.kBrushless);
    kickupMotor = new SparkFlex(kickupID, MotorType.kBrushless);

    leftConfig = new SparkFlexConfig();
    rightConfig = new SparkFlexConfig();
    kickupConfig = new SparkFlexConfig();

    rightConfig.idleMode(IdleMode.kBrake);
    rightMotor.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    leftConfig.inverted(inverted);
    leftConfig.idleMode(IdleMode.kBrake);
    leftMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    kickupConfig.idleMode(IdleMode.kBrake);
    kickupMotor.configure(kickupConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setIndexSpeed(double speed) {
    rightMotor.set(speed);
    leftMotor.set(speed);
  }

  public void turnKickup() {
    kickupMotor.set(kickupSpeed);
  }

  public void setKickupSpeed(double speed) {
    kickupSpeed = speed;
  }

  public SparkFlex getRightMotor() {
    return rightMotor;
  }

  public SparkFlex getLeftMotor() {
    return leftMotor;
  }

  public SparkFlex getKickupMotor() {
    return kickupMotor;
  }

  public SparkFlexConfig getRightConfig() {
    return rightConfig;
  }

  public SparkFlexConfig getLeftConfig() {
    return leftConfig;
  }

  public SparkFlexConfig getKickupConfig() {
    return kickupConfig;
  }

  public double getKickupSpeed() {
    return kickupSpeed;
  }
}
