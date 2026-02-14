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
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.ProfiledPIDController;
import frc.robot.util.ClosedLoopConstants;
import frc.robot.util.PIDLogger;

public class IntakeSubsystem extends SubsystemBase {

  private double intakeSpeed = 0.5;

private final ClosedLoopConstants defaultPIDConstants = new ClosedLoopConstants(
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0
);

  private ProfiledPIDController pivotPID = new ProfiledPIDController(
    defaultPIDConstants.kP, 
    defaultPIDConstants.kI, 
    defaultPIDConstants.kD,
    new TrapezoidProfile.Constraints(
        defaultPIDConstants.maxVelocity, 
        defaultPIDConstants.maxAcceleration
    )
  );

  private ArmFeedforward pivotFeedForward = new ArmFeedforward(
    defaultPIDConstants.kG,
    defaultPIDConstants.kS,
    defaultPIDConstants.kV,
    defaultPIDConstants.kA
  );

private final PIDLogger pidLogger = new PIDLogger(
        "Subsystems/" + getName(),
        defaultPIDConstants,
        constants -> {
            this.pivotPID.setPID(constants.kP, constants.kI, constants.kD);
            this.pivotPID.setConstraints(
               new TrapezoidProfile.Constraints(constants.maxVelocity, constants.maxAcceleration)
            );
            this.pivotFeedForward.setKa(constants.kA);
            this.pivotFeedForward.setKs(constants.kS);
            this.pivotFeedForward.setKv(constants.kV);
       }
    );
  
  private SparkFlex wheelMotor = new SparkFlex(Constants.IntakeConstants.WHEEL_ID, MotorType.kBrushless);
  private SparkFlex pivotMotor = new SparkFlex(Constants.IntakeConstants.PIVOT_ID, MotorType.kBrushless);


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
  }
  
  //Probably not needed
  public void setPivotSpeed(double speed) { 
    pivotMotor.set(speed);
  }

  public double getPivotVelocity() { 
    return pivotMotor.getAbsoluteEncoder().getVelocity();
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
      return pivotMotor.getEncoder().getVelocity();
  }

  public void setIntakeWheelSpeed(double speed) {
    intakeSpeed = speed;
    wheelMotor.set(speed);
  }

  public double closedLoopCalculate(double target, double nextVelocity) {
    double pidOutput = pivotPID.calculate(target);
    double ffOutput = pivotFeedForward.calculateWithVelocities(this.getPivotPositionInDegrees(), this.getPivotVelocity(), nextVelocity);
    return pidOutput + ffOutput;
  }

  @Override
  public void periodic() {
  }
}
