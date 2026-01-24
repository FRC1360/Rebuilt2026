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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class IndexSubsystem extends SubsystemBase {

  private SparkFlex indexMotor;
  private SparkFlexConfig indexConfig;

  /** Creates a new IndexSubsystem. */

  public IndexSubsystem() {
    indexMotor = new SparkFlex(Constants.IndexConstants.hopperConveyorID, MotorType.kBrushless);
    indexConfig = new SparkFlexConfig();

    indexConfig.idleMode(IdleMode.kBrake);
    indexMotor.configure(indexConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setIndexVoltage(double volts) {
    indexMotor.setVoltage(volts);
  }

  public void setIndexSpeed(double speed) {
    indexMotor.set(speed);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Subsystems/IndexSubsystem/indexVoltage", indexMotor.getBusVoltage());
  }
}
