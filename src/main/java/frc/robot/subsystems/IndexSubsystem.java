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
import frc.robot.Constants.IndexConstants;
import edu.wpi.first.wpilibj.DigitalInput;

public class IndexSubsystem extends SubsystemBase {

    private final SparkFlex hopperMotor;
    private SparkFlexConfig hopperMotorConfig;
    private final SparkFlex magazineMotor;
    private SparkFlexConfig magazineMotorConfig;
    private final DigitalInput magazineSensor;
    public final Trigger magazineSensorTriggered;

    /** Creates a new IndexSubsystem. */

    public IndexSubsystem() {
        hopperMotor = new SparkFlex(IndexConstants.HOPPER_VORTEX_CAN_ID, MotorType.kBrushless);
        hopperMotorConfig = new SparkFlexConfig();
        magazineMotor = new SparkFlex(IndexConstants.MAGAZINE_VORTEX_CAN_ID, MotorType.kBrushless);
        magazineMotorConfig = new SparkFlexConfig();

        hopperMotor.clearFaults();
        hopperMotorConfig.idleMode(IdleMode.kBrake);
        hopperMotorConfig.inverted(IndexConstants.HOPPER_VORTEX_INVERTED);
        hopperMotorConfig.smartCurrentLimit(
                IndexConstants.HOPPER_VORTEX_STALL_CURRENT_LIMIT,
                IndexConstants.HOPPER_VORTEX_FREE_CURRENT_LIMIT);
        hopperMotor.configure(hopperMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        magazineMotor.clearFaults();
        magazineMotorConfig.idleMode(IdleMode.kBrake);
        magazineMotorConfig.inverted(IndexConstants.MAGAZINE_VORTEX_INVERTED);
        magazineMotorConfig.smartCurrentLimit(
                IndexConstants.MAGAZINE_VORTEX_STALL_CURRENT_LIMIT,
                IndexConstants.MAGAZINE_VORTEX_FREE_CURRENT_LIMIT);
        magazineMotor.configure(magazineMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        this.magazineSensor = new DigitalInput(IndexConstants.MAGAZINE_SENSOR_DIGITAL_CHANNEL);
        this.magazineSensorTriggered = new Trigger((() -> (!magazineSensor.get())));
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
