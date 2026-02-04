// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class HoodSubsystem extends SubsystemBase {

    private final TalonFX hoodMotor = new TalonFX(50);
    private final VoltageOut hoodVoltageRequest = new VoltageOut(0.0);

    private final FeedbackConfigs motorFeedbackConfigs;
    private final MotorOutputConfigs motorOutputConfigs;

    private final double kConversonFactor = ((164.0 / 12.0) * (60.0 / 18.0)) / 360.0;
    private final double kInitialAngle = 75.0;
     /** Creates a new HoodSubsystem. */
    public HoodSubsystem() {
        motorFeedbackConfigs = new FeedbackConfigs()
            .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
            .withSensorToMechanismRatio(kConversonFactor);

        motorOutputConfigs = new MotorOutputConfigs()
            .withInverted(InvertedValue.Clockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake);

        hoodMotor.getConfigurator().apply(motorFeedbackConfigs);
        hoodMotor.getConfigurator().apply(motorOutputConfigs);
        hoodMotor.setPosition(kInitialAngle);
    }

    @Override
    public void periodic() { 
    }

    public void setHoodMotorVoltage(double volts) {
        hoodMotor.setVoltage(volts);
    }

      public double getCurrentAngle() {
        return hoodMotor.getPosition().getValueAsDouble();
    }
    
    public double getCurrentVelocity() {
        return hoodMotor.getVelocity().getValueAsDouble();
    }
      private SysIdRoutine sysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(
            Volts.of(0.1).per(Second),  // Ramp Rate of 0.1V/s
            Volts.of(0.4),              // Dynamic Step Voltage of 0.4V
            Seconds.of(10),                         // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("Hood_SysID_State", state.toString())
            //tHood angle, aHood angle, tFlywheel speed, aFlywheel speed
        ),
        new SysIdRoutine.Mechanism(
         (volts) -> hoodMotor.setControl(hoodVoltageRequest.withOutput(volts.in(Volts))),
         null,
         this
      )
    );

      public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
       return sysIdRoutine.quasistatic(direction);
    }

     public Command sysIdDynamic(SysIdRoutine.Direction direction) {
       return sysIdRoutine.dynamic(direction);
    }
}