// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class TurretSubsystem extends SubsystemBase {

   private final SparkMax motor;  
   private final EncoderConfig encoderConfig; 
   private final SparkMaxConfig motorConfig;
   private static final int TurretMotorID = 0;
   private static final double gearRatio = (1.0 / 7.0) * 360.0;
   private final SysIdRoutine routine;   
      
   private final NetworkTable loggingTable;
   private final DoublePublisher turretPosDoublePublisher;

   public TurretSubsystem() { 
      motor = new SparkMax(TurretMotorID, MotorType.kBrushless);
      motorConfig = new SparkMaxConfig(); 
      encoderConfig = new EncoderConfig();
         
      encoderConfig.velocityConversionFactor(gearRatio / 60.0); // per minute to per second
      encoderConfig.positionConversionFactor(gearRatio); 
      motorConfig.idleMode(IdleMode.kBrake);
      motorConfig.apply(encoderConfig);
            
      motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

      routine = new SysIdRoutine(
      new SysIdRoutine.Config(
        Volts.of(0.5).per(Second),
        Volts.of(3.5),
        Seconds.of(10)
      ),
      new SysIdRoutine.Mechanism(
        (voltage) -> motor.setVoltage(voltage),
        // Tell SysId how to record a frame of data for each motor on the mechanism being
        log -> {
          // Record a frame for the turret motor.
          log.motor("turret")
              .voltage(Volts.of(motor.getAppliedOutput() * RobotController.getBatteryVoltage()))
              .angularPosition(Angle.ofBaseUnits(motor.getEncoder().getPosition(), Degrees))
              .angularVelocity(AngularVelocity.ofBaseUnits(motor.getEncoder().getVelocity(), Degrees.per(Second)));
        },
        // Tell SysId to make generated commands require this subsystem, suffix test state in
        // WPILog with this subsystem's name ("turret")
        this));
        
        loggingTable = NetworkTableInstance.getDefault().getTable("Subsystems/" + getName());
        turretPosDoublePublisher = loggingTable.getDoubleTopic("Current position").publish(); 
      }
      
   public void setVoltage(double volts) {
      motor.setVoltage(volts);
   }

   @Override
   public void periodic() {
      turretPosDoublePublisher.accept(motor.getEncoder().getPosition());
   }
   
   public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return routine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) { 
    return routine.dynamic(direction);
  }
}