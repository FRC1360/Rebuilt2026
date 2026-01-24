// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import com.revrobotics.spark.SparkMax;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;


 
public class HoodSubsystem extends SubsystemBase {

    private final TalonFX hoodMotor;

    private final FeedbackConfigs motorFeedbackConfigs;
    private final MotorOutputConfigs motorOutputConfigs;

    private final double kConversonFactor = (164.0 / 12.0) / 360.0;
    private final double kInitialAngle = 75.0;

  
  /** Creates a new HoodSubsystem. */
  
    public HoodSubsystem() {
        hoodMotor = new TalonFX(50);

        motorFeedbackConfigs = new FeedbackConfigs()
            .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
            .withSensorToMechanismRatio(kConversonFactor);

        motorOutputConfigs = new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake);

        hoodMotor.getConfigurator().apply(motorFeedbackConfigs);
        hoodMotor.getConfigurator().apply(motorOutputConfigs);
        hoodMotor.setPosition(kInitialAngle);
    }

    @Override
    public void periodic() { 
        SmartDashboard.putNumber("Subsystems/HoodSubsystem/Current Position Degrees", hoodMotor.getPosition().getValueAsDouble());
    }

    public void setHoodMotorVoltage(double volts) {
        hoodMotor.setVoltage(volts);
    }

    public double getCurrentAngle() {
        return hoodMotor.getPosition().getValueAsDouble();
    }
}

