// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.util.FieldConstants;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

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
   private static final int TurretMotorID = 60;
   private static final double gearRatio = (1.0 / 7.0) * 360.0;
   private final SysIdRoutine routine;   
      
   private final NetworkTable loggingTable;
   private final DoublePublisher turretPosDoublePublisher;
   private final DoublePublisher turretVelDoublePublisher;
   private final DoublePublisher turretVoltageDoublePublisher;
   private final StructPublisher<Pose2d> hubPosePublisher;

   private final double initialAngle = 0;

   private final OrbitCamera orbitCamera;
   private Pose2d estimatedPose;

   public TurretSubsystem() { 
      motor = new SparkMax(TurretMotorID, MotorType.kBrushless);
      motorConfig = new SparkMaxConfig(); 
      encoderConfig = new EncoderConfig();
         
      encoderConfig.velocityConversionFactor(gearRatio / 60.0); // per minute to per second
      encoderConfig.positionConversionFactor(gearRatio); 
      motorConfig.idleMode(IdleMode.kBrake);
      motorConfig.apply(encoderConfig);
            
      motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      motor.getEncoder().setPosition(initialAngle);

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
        turretPosDoublePublisher = loggingTable.getDoubleTopic("Current Position").publish(); 
        turretVelDoublePublisher = loggingTable.getDoubleTopic("Current Velocity").publish(); 
        turretVoltageDoublePublisher = loggingTable.getDoubleTopic("Current Voltage").publish(); 
        hubPosePublisher = loggingTable.getStructTopic("Hub Pose", Pose2d.struct).publish(); 

        hubPosePublisher.accept(FieldConstants.redAllianceHubPose);

        orbitCamera = new OrbitCamera(
         new Transform3d(
            new Translation3d(0.189, 0, 0.505),
            new Rotation3d(0, -Math.toRadians(25), Math.toRadians(0))
         ), "photoncamera_turret");
      }
      
   public void setVoltage(double volts) {
      motor.setVoltage(volts);
   }

   public double getCurrentVelocity(){
      return motor.getEncoder().getVelocity();
   }

   public double getCurrentAngle(){
      return motor.getEncoder().getPosition();
   }

   @Override
   public void periodic() {
      turretPosDoublePublisher.accept(motor.getEncoder().getPosition());
      turretVelDoublePublisher.accept(motor.getEncoder().getVelocity());
      turretVoltageDoublePublisher.accept(motor.getAppliedOutput() * RobotController.getBatteryVoltage());

      this.updatePose();
   }
   
   private void updatePose() {
      orbitCamera.updatePipelineResults();
      Optional<EstimatedRobotPose> visionEst = Optional.empty();
        for (var result : orbitCamera.getPipelineResults()) {
            visionEst = orbitCamera.getPhotonPoseEstimator().estimateCoprocMultiTagPose(result);
            if (visionEst.isEmpty()) {
                visionEst = orbitCamera.getPhotonPoseEstimator().estimateLowestAmbiguityPose(result);
            }
            orbitCamera.updateEstimationStdDevs(visionEst, result.getTargets());

            visionEst.ifPresent(
                    est -> {
                        // Change our trust in the measurement based on the tags we can see
                        var estStdDevs = orbitCamera.getEstimationStdDevs();

                        orbitCamera.updateStructPublisher(est.estimatedPose.toPose2d());
                        this.estimatedPose = est.estimatedPose.toPose2d();
                    });
        }
   }

   public Pose2d getEstimatedPose() {
      return this.estimatedPose;
   }
   
   public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return routine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) { 
    return routine.dynamic(direction);
  }
}