// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;

import com.ctre.phoenix6.hardware.TalonFX;


 
public class HoodSubsystem extends SubsystemBase {
  private TalonFX hoodmotor;
  private int hoodMotorID = 0;
  private int gearRatio = 0;
  private TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();
  private Slot0Configs slot0Configs = talonFXConfigs.Slot0;

  
  /** Creates a new HoodSubsystem. */

  
  public HoodSubsystem() {
    hoodmotor = new TalonFX(hoodMotorID);
    var hoodMConfigs = new TalonFXConfiguration();

    slot0Configs.kP = 0.0; // An error of 1 rotation results in 2.4 V output
    slot0Configs.kI = 0.0; // no output for integrated error
    slot0Configs.kD = 0.0;     // A velocity of 1 rps results in 0.1 V output
    slot0Configs.kS = 0.0;
    slot0Configs.kV = 0.0;
    slot0Configs.kG = 0.0;
    slot0Configs.kA = 0.0;

    MotionMagicConfigs hoodConfigs = hoodMConfigs.MotionMagic;
    hoodConfigs.MotionMagicCruiseVelocity = 0.0;
    hoodConfigs.MotionMagicExpo_kV = 0.0;
    hoodConfigs.MotionMagicExpo_kA = 0.0;
    hoodConfigs.MotionMagicJerk = 0.0;
    
  }

  void MoveToAngle(double angle,double speed){
    
    MotionMagicVoltage m_request = new MotionMagicVoltage(0); 
    double rotation = (angle/360)*gearRatio; // turn degrees into rotation for built in PID
    hoodmotor.setControl(m_request.withPosition(rotation));
  }


  @Override
  public void periodic() {
    
    // This method will be called once per scheduler run
    

  }
}

