// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.PIDLogger;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;

public class IntakeSubsystem extends SubsystemBase {
    
    public BooleanSupplier intakeWheelsEnabled;
    public BooleanSupplier intakeWheelsDisabled;
    private PIDLogger pidLogger;

    private double maxAccel = 0.0;
    private double maxVel = 0.0;
    private TrapezoidProfile.Constraints pivotConstraints = new Constraints(maxVel, maxAccel);
    private double kP = 0.0;
    private double kI = 0.0;
    private double kD = 0.0;
    private ProfiledPIDController pivotPID = new ProfiledPIDController(kP, kI, kD, pivotConstraints);
    private double kG = 0.0;
    private double kS = 0.0;
    private double kA = 0.0;
    private double kV = 0.0;
    private double currentAngle;
    private double targetAngle;

    private SparkFlex wheelMotor = new SparkFlex(Constants.IntakeConstants.WHEEL_ID, MotorType.kBrushless);
    private SparkFlex pivotMotor = new SparkFlex(Constants.IntakeConstants.PIVOT_ID, MotorType.kBrushless);

    private ArmFeedforward pivotFeedForward = new ArmFeedforward(kG, kS, kV, kA);

    SparkFlexConfig wheelConfig = new SparkFlexConfig();
    SparkFlexConfig pivotConfig = new SparkFlexConfig();

    public IntakeSubsystem() {
        wheelMotor.clearFaults();
        pivotMotor.clearFaults();

        
        wheelConfig.inverted(false);
        wheelConfig.idleMode(IdleMode.kCoast);
        wheelConfig.smartCurrentLimit(30, 30);

        wheelMotor.configure(
            wheelConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );

        pivotConfig.inverted(false);
        pivotConfig.idleMode(IdleMode.kBrake);
        pivotConfig.smartCurrentLimit(30, 30);

        pivotMotor.configure(
            pivotConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );


        this.currentAngle = Constants.IntakeConstants.HOME_ANGLE;

        this.intakeWheelsDisabled = () -> {
            return (pivotPID.atSetpoint() && 
            Math.abs(currentAngle - Constants.IntakeConstants.HOME_ANGLE) <= Constants.IntakeConstants.ANGLE_TOLERANCE);
        };
        
        // if (false) {intakeWheelsEnabled = true;} 
        // else {intakeWheelsEnabled = false;}
        
        this.intakeWheelsEnabled = () -> {
            return (pivotPID.atSetpoint() && 
            Math.abs(currentAngle - Constants.IntakeConstants.DEPLOYED_ANGLE)<= Constants.IntakeConstants.ANGLE_TOLERANCE);
        };
    }
  
  //Probably not needed
  public void setPivotSpeed(double speed) { 
    pivotMotor.set(speed);
  }

  public void setPivotVoltage(double voltage) {
    pivotMotor.setVoltage(voltage);
  }

  public double getPivotVelocity() { 
    return pivotMotor.getEncoder().getVelocity();
  }

  public ProfiledPIDController getPidController () {
    return this.pivotPID;
  }

  public double getPivotPosition() {
    return pivotMotor.getEncoder().getPosition();
  }

  public ArmFeedforward getfeedForward () {
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
      return pivotMotor.getEncoder().getPosition() * 360.0;
  }

  public double getWheelSpeed() {
      return wheelMotor.getEncoder().getVelocity();
  }

  public void setIntakeWheelSpeed(double speed) {
    wheelMotor.set(speed);
  }

  public double closedLoopCalculate(double target, double nextVelocity) {
    this.targetAngle = target;
    this.pivotPID.setGoal(targetAngle);
    double pidOutput = pivotPID.calculate(targetAngle);
    double ffOutput = pivotFeedForward.calculateWithVelocities(pidOutput, getPivotVelocity(), nextVelocity);
    return pidOutput + ffOutput;
  }
   
  public double getCurrentAngle() {
    return this.currentAngle;
  }   

      public void grabConstantsFromNetworkTables() {
        this.pidLogger.updateConstants();
    }

    public void resetPIDController() {
        pivotPID.reset(
            this.getCurrentAngle(),
            this.getPivotVelocity()
        );
    }

  @Override
  public void periodic() {
    currentAngle = getPivotPositionInDegrees();
  }
}
