// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IndexSubsystem extends SubsystemBase {
  /** Creates a new IndexSubsystem. */

  SparkFlex rightMotor;
  SparkFlex leftMotor;
  int rightMotorID = 0;
  int leftMotorID = 0;
  SparkMaxConfig leftConfig;
  SparkMaxConfig rightConfig;
  boolean inverted;

  public IndexSubsystem() {
    rightMotor = new SparkFlex(rightMotorID, MotorType.kBrushless);
    leftMotor = new SparkFlex(leftMotorID, MotorType.kBrushless);
    leftConfig = new SparkMaxConfig();
    rightConfig = new SparkMaxConfig();

    leftConfig.inverted(inverted);
    rightConfig.idleMode(IdleMode.kBrake);
    leftMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    rightConfig.idleMode(IdleMode.kBrake);
    rightMotor.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setMotorSpeed(double speed) {
    rightMotor.set(speed);
    leftMotor.set(speed);
  }
}
