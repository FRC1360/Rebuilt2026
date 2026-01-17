// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.hardware.TalonFX;


public class TurretSubsystem extends SubsystemBase {
  private final TalonFX turretMotor = new TalonFX(Constants.TurretConstants.TurretMotorID);
  private final MotionMagicVoltage turretRequest = new MotionMagicVoltage(0);
  private double targetAngle;
  
  public TurretSubsystem() { 
    var turretConfigs = new TalonFXConfiguration();

    Slot0Configs slot0Configs = turretConfigs.Slot0;
    slot0Configs.kS = 0.0;
    slot0Configs.kV = 0.0;
    slot0Configs.kA = 0.0;
    slot0Configs.kP = 0.0;
    slot0Configs.kI = 0.0;
    slot0Configs.kD = 0.0;

    MotionMagicConfigs turrConfigs = turretConfigs.MotionMagic;
    turrConfigs.MotionMagicCruiseVelocity = 0.0;
    turrConfigs.MotionMagicExpo_kV = 0.0;
    turrConfigs.MotionMagicExpo_kA = 0.0;
    turrConfigs.MotionMagicJerk = 0.0;
    turretMotor.getConfigurator().apply(turrConfigs);

    targetAngle = 0.0;
  }
  
  public void setAngle(double angle) {
    this.targetAngle = angle;
    turretMotor.setControl(turretRequest.withPosition(this.targetAngle / 360)); //Converts rotations to angle
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Current Turret Target Angle", targetAngle);
  }
}