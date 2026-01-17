// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class IndexSubsystem extends SubsystemBase {
  /** Creates a new IndexSubsystem. */

  SparkFlex rightMotor;
  SparkFlex leftMotor;

  public IndexSubsystem() {
    rightMotor = new SparkFlex(0, MotorType.kBrushless);
    leftMotor = new SparkFlex(0, MotorType.kBrushless);
    
    leftMotor.setInverted(true);
  }

  public void setMotorSpeed(double speed) {
    rightMotor.set(speed);
    leftMotor.set(speed);
  }
}
