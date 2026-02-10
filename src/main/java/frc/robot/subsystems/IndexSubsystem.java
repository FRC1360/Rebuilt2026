// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.DigitalInput;

public class IndexSubsystem extends SubsystemBase {

  private SparkFlex hopperMotor;
  private SparkFlexConfig hopperMotorConfig;
  private SparkFlex magazineMotor;
  private SparkFlexConfig magazineMotorConfig;
  private DigitalInput magazineSensor;
  public Trigger magazineSensorTriggered;

  /** Creates a new IndexSubsystem. */

  public IndexSubsystem() {
    hopperMotor = new SparkFlex(Constants.IndexConstants.HOPPER_CONVEYOR_ID, MotorType.kBrushless);
    hopperMotorConfig = new SparkFlexConfig();

    hopperMotorConfig.idleMode(IdleMode.kBrake);
    hopperMotor.configure(hopperMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    magazineMotor = new SparkFlex(Constants.IndexConstants.MAGAZINE_CONVEYOR_ID, MotorType.kBrushless);
    magazineMotorConfig = new SparkFlexConfig();

    magazineMotorConfig.idleMode(IdleMode.kBrake);
    magazineMotor.configure(magazineMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    this.magazineSensor = new DigitalInput(Constants.IndexConstants.MAGAZINE_SENSOR_ID);
    this.magazineSensorTriggered = new Trigger(() -> (magazineSensor.get()));
  }

  public void setHopperVoltage(double volts) {
    hopperMotor.setVoltage(volts);
  }

  public void setHopperSpeed(double speed) {
    hopperMotor.set(speed);
  }

    public void setMagazineVoltage(double volts) {
    magazineMotor.setVoltage(volts);
  }

  public void setMagazineSpeed(double speed) {
    magazineMotor.set(speed);
  }

  @Override
  public void periodic() {
  }
}
