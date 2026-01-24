// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;

public class IntakeSubsystem extends SubsystemBase {

  private double intakeSpeed = 0.5;

  private double kP = 0.0;
  private double kI = 0.0;
  private double kD = 0.0;
  private PIDController pivotPID = new PIDController(kP, kI, kD);

  private double kG = 0.0;
  private double kS = 0.0;
  private double kA = 0.0;
  private double kV = 0.0;

  private SparkFlex wheel = new SparkFlex(Constants.IntakeConstants.WHEEL_ID, MotorType.kBrushless);
  private SparkFlex pivot = new SparkFlex(Constants.IntakeConstants.PIVOT_ID, MotorType.kBrushless);

  private ArmFeedforward pivotFeedForward = new ArmFeedforward(kG, kS, kV, kA);

  SparkFlexConfig wheelConfig = new SparkFlexConfig();

  public IntakeSubsystem() {
       wheel.clearFaults();
  pivot.clearFaults();

  wheelConfig.inverted(false);
  wheelConfig.idleMode(IdleMode.kCoast);
  wheelConfig.smartCurrentLimit(30);

  wheel.configure(
      wheelConfig,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters
  );
  }
  

  //Probably not needed
  public void setPivotSpeed(double speed) { 
    pivot.set(speed);
  }

  public PIDController getPidController () {
    return this.pivotPID;
  }


  public double getPivotPosition() {
    return pivot.getEncoder().getPosition();
  }

  public ArmFeedforward  getfeedForward () {
    return this.pivotFeedForward;
  }

  public void setFeedForwardParameters(double passed_kG, double passed_kS, double passed_kA, double passed_kV) {
    this.pivotFeedForward.setKg(passed_kG);
    this.pivotFeedForward.setKs(passed_kS);
    this.pivotFeedForward.setKa(passed_kA);
    this.pivotFeedForward.setKv(passed_kV);
  }

  public void setPIDControllerParameters(double passed_kP, double passed_kI, double passed_kD) {
    this.pivotPID.setP(passed_kP);
    this.pivotPID.setI(passed_kI);
    this.pivotPID.setD(passed_kD);
  } 

  public double getPivotPositionInDegrees() {
      return pivot.getEncoder().getPosition() * 360.0;
  }


  public double getWheelSpeedInRPM() {
      return pivot.getEncoder().getVelocity();
  }

  public void setIntakeWheelSpeed(double speed) {
    intakeSpeed = speed;
    wheel.set(speed);
  }

  @Override
  public void periodic() {
      
      SmartDashboard.putNumber("SubsystemIntakePivotPosition", getPivotPositionInDegrees());

      SmartDashboard.putNumber("SubsystemIntakeWheelSpeed", getWheelSpeedInRPM());
  }
}

