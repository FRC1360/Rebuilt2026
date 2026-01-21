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
  /** Creates a new IndexSubsystem. */

  private SparkFlex hopperConveyorMotor;
  private SparkFlex magazineConveyorMotor;
  private SparkFlex magazineRollerMotor;

  private SparkFlexConfig hopperConveyorConfig;
  private SparkFlexConfig magazineConveyorConfig;
  private SparkFlexConfig magazineRollerConfig;

  public IndexSubsystem() {
    hopperConveyorMotor = new SparkFlex(Constants.IndexConstants.hopperConveyorID, MotorType.kBrushless);
    magazineConveyorMotor = new SparkFlex(Constants.IndexConstants.magazineConveyorID, MotorType.kBrushless);
    magazineRollerMotor = new SparkFlex(Constants.IndexConstants.magazineRollerID, MotorType.kBrushless);

    hopperConveyorConfig = new SparkFlexConfig();
    magazineConveyorConfig = new SparkFlexConfig();
    magazineRollerConfig = new SparkFlexConfig();

    magazineConveyorConfig.idleMode(IdleMode.kBrake);
    hopperConveyorMotor.configure(magazineConveyorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    hopperConveyorConfig.idleMode(IdleMode.kBrake);
    magazineConveyorMotor.configure(hopperConveyorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    magazineRollerConfig.idleMode(IdleMode.kBrake);
    magazineRollerMotor.configure(magazineRollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setHopperConveyorSpeed(double speed) {
    hopperConveyorMotor.set(speed);
    SmartDashboard.putNumber("Subsystems/IndexSubsystem/hopperConveyorSpeed", speed);
  }

  public void setMagazineConveyorSpeed(double speed) {
    magazineConveyorMotor.set(speed);
    SmartDashboard.putNumber("Subsystems/IndexSubsystem/magazineConveyorSpeed", speed);
  }

  public void setMagazineRollerSpeed(double speed) {
    magazineRollerMotor.set(speed);
    SmartDashboard.putNumber("Subsystems/IndexSubsystem/magazineRollerSpeed", speed);
  }
}
