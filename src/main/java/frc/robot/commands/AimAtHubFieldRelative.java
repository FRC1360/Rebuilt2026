// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.photonvision.PhotonUtils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.TurretSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AimAtHubFieldRelative extends Command {
  private Pose2d robotPose;  // Robot Pose on the field, from Odometry
  private Pose2d turretFieldPose;  // Turret Pose on the field, will calculate down below
  private Pose2d hubPose;  // Hub Pose on the field, get from CAD / WPILib
  private Transform2d turretToRobotInitialTransform;  // Initial transform of the turret relative to the robot, including the rotation on startup. From CAD, assuming the intake is the front, this would be (-0.1397, 0.1397, 90deg)
  private Rotation2d turretLocalAngle;
  private Rotation2d targetYaw;  // Zero Yaw = Facing the Hub
  private Rotation2d currentYaw; 
  private TurretSubsystem m_turretSubsystem;
  private double pidControllerOutput;
  private double feedForwardControllerOutput;
  private double wrappedTarget;

  private final NetworkTable loggingTable = NetworkTableInstance.getDefault().getTable("Commands/"+getName());
  private final DoublePublisher turretCurrentYawPublisher = loggingTable.getDoubleTopic("Turret Current Yaw Publisher").publish();
  private final DoublePublisher turretTargetYawPublisher = loggingTable.getDoubleTopic("Target Angle Publisher").publish();
  private final StructPublisher<Pose2d> robotPosePublisher = loggingTable.getStructTopic("Robot Pose", Pose2d.struct).publish();
  private final StructPublisher<Pose2d> turretPosePublisher = loggingTable.getStructTopic("turret Pose", Pose2d.struct).publish();


  /** Creates a new AimAtHubFieldRelative. */
  public AimAtHubFieldRelative(CommandSwerveDrivetrain drivetrain, TurretSubsystem turretSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_turretSubsystem = turretSubsystem;
    this.robotPose = drivetrain.samplePoseAt(Timer.getFPGATimestamp()).get();
    // Used turnary for a good reason ;)
    this.hubPose = (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue ? Constants.TurretConstants.FIELD_RELATIVE_BLUE_HUB_POS : Constants.TurretConstants.FIELD_RELATIVE_RED_HUB_POS);
    this.turretToRobotInitialTransform = new Transform2d(0.23876, 0.0, new Rotation2d());
    this.turretLocalAngle = m_turretSubsystem.getCurrentRotation();
    this.turretFieldPose = new Pose2d();
    this.currentYaw = new Rotation2d();
    // this.currentYaw = PhotonUtils.getYawToPose(turretFieldPose, hubPose).getDegrees();
    this.targetYaw= new Rotation2d();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.turretFieldPose = robotPose.plus(turretToRobotInitialTransform).plus(new Transform2d(new Translation2d(), turretLocalAngle));
    this.currentYaw = Rotation2d.fromDegrees(m_turretSubsystem.getCurrentAngle());
    this.targetYaw = currentYaw.minus(PhotonUtils.getYawToPose(turretFieldPose, hubPose));
    this.wrappedTarget = m_turretSubsystem.calculateWrapAround(targetYaw);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.turretLocalAngle = m_turretSubsystem.getCurrentRotation();
    this.turretFieldPose = robotPose.plus(turretToRobotInitialTransform).plus(new Transform2d(new Translation2d(), turretLocalAngle));
    this.currentYaw = Rotation2d.fromDegrees(m_turretSubsystem.getCurrentAngle());
    //this.targetYaw = currentYaw.minus(PhotonUtils.getYawToPose(turretFieldPose, hubPose));
    this.targetYaw = PhotonUtils.getYawToPose(turretFieldPose, hubPose);
    this.wrappedTarget = targetYaw.getDegrees();
    robotPosePublisher.accept(robotPose);
    turretPosePublisher.accept(turretFieldPose);
    //this.targetYaw = m_turretSubsystem.calculateWrapAround(currentYaw.minus(PhotonUtils.getYawToPose(turretFieldPose, hubPose)));

    
    //pidControllerOutput = m_turretSubsystem.m_pidController.calculate(m_turretSubsystem.getCurrentAngle(), wrappedTarget);
    pidControllerOutput = m_turretSubsystem.m_pidController.calculate(-this.targetYaw.getDegrees(), 0.0);
    feedForwardControllerOutput = m_turretSubsystem.m_feedForward.calculate(m_turretSubsystem.m_pidController.getSetpoint().velocity);


    m_turretSubsystem.setVoltage(pidControllerOutput + feedForwardControllerOutput);

    
    //turretCurrentYawPublisher.accept(this.m_turretSubsystem.getCurrentAngle());
    turretCurrentYawPublisher.accept(-this.targetYaw.getDegrees());
    turretTargetYawPublisher.accept(m_turretSubsystem.m_pidController.getGoal().position);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
