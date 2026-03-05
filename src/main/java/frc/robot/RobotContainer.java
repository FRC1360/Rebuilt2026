// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import org.photonvision.PhotonUtils;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.flywheel.SetFlywheelVelocityFromPoseCommand;
import frc.robot.commands.hood.SetHoodAngleCommand;
import frc.robot.commands.hood.SetHoodAngleFromPose;
import frc.robot.commands.index.ActivateAgitatedIndexCommand;
import frc.robot.commands.index.SetIndexSpeedsCommand;
import frc.robot.commands.intake.DeployIntakeCommand;
import frc.robot.commands.intake.RetractIntakeCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.util.FieldConstants;
import frc.robot.util.RobotState;

public class RobotContainer {

    private final NetworkTable loggingTable = NetworkTableInstance.getDefault().getTable("Robot Container");
    private final BooleanPublisher readyToShootEntry = loggingTable.getBooleanTopic("Ready To Shoot").publish();

    private final RobotState robotState = RobotState.getInstance();

    // Replace with CommandPS4Controller or CommandJoystick if needed
    private final CommandXboxController m_controller = new CommandXboxController(0);
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
    private final IndexSubsystem m_indexSubsystem = new IndexSubsystem();
    private final FlywheelSubsystem m_flywheelSubsystem = new FlywheelSubsystem();
    private final HoodSubsystem m_HoodSubsystem = new HoodSubsystem();

    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top
                                                                                        // speed
    private double DriveSpeedWhileIntaking = 0.3 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second
                                                                                      // max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.15).withRotationalDeadband(MaxAngularRate * 0.15) // Add a 15% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.FieldCentricFacingAngle driveFacingAngle = new SwerveRequest.FieldCentricFacingAngle()
            .withDeadband(MaxSpeed * 0.15) // Add a 15% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withHeadingPID(5.0, 0.0, 0.0); // Use open-loop control for drive motors
    private final SwerveTelemetry swerveLogger = new SwerveTelemetry(MaxSpeed);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        robotState.setAllSuppliers(
                () -> drivetrain.getState().Pose,
                () -> new Rotation2d());

        configureBindings();
        drivetrain.registerTelemetry(swerveLogger::telemeterize);

        // Run warmup command for pathplanner as per CTRE example
        CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());
    }

    private void configureBindings() {
        // Driving
        drivetrain.setDefaultCommand(
                drivetrain.applyRequest(() -> drive.withVelocityX(-m_controller.getLeftY() * MaxSpeed)
                        .withVelocityY(-m_controller.getLeftX() * MaxSpeed)
                        .withRotationalRate(-m_controller.getRightX() * MaxAngularRate)));
        m_controller.b().onTrue(Commands.runOnce(() -> drivetrain.seedFieldCentric(), drivetrain));
        m_controller.leftTrigger(0.8).whileTrue(
                drivetrain.applyRequest(() -> drive.withVelocityX(-m_controller.getLeftY() * DriveSpeedWhileIntaking)
                        .withVelocityY(-m_controller.getLeftX() * DriveSpeedWhileIntaking)
                        .withRotationalRate(-m_controller.getRightX() * MaxAngularRate)));

        // Intaking
        m_intakeSubsystem.setDefaultCommand(new RetractIntakeCommand(m_intakeSubsystem));
        m_controller.a()
                .toggleOnTrue(new DeployIntakeCommand(m_intakeSubsystem, m_controller.leftTrigger(0.8)));

        // Shooting
        Trigger readyToShoot = m_flywheelSubsystem.flywheelAtTarget.and(m_HoodSubsystem.hoodAtTarget)
                .and(() -> Math.abs(driveFacingAngle.HeadingController.getPositionError()) < Units
                        .degreesToRadians(2.0));
        Command prepareToShoot = new SetFlywheelVelocityFromPoseCommand(m_flywheelSubsystem,
                FieldConstants.RED_ALLIANCE_HUB_POSE)
                .alongWith(new SetHoodAngleFromPose(m_HoodSubsystem, FieldConstants.RED_ALLIANCE_HUB_POSE))
                .alongWith(drivetrain.applyRequest(() -> driveFacingAngle.withTargetDirection(
                        PhotonUtils.getYawToPose(
                                new Pose2d(
                                        robotState.getTurretOdomPose().getTranslation(),
                                        new Rotation2d()),
                                FieldConstants.RED_ALLIANCE_HUB_POSE).plus(Rotation2d.fromDegrees(90)))
                        .withVelocityX(-m_controller.getLeftY() * MaxSpeed)
                        .withVelocityY(-m_controller.getLeftX() * MaxSpeed)));

        m_flywheelSubsystem.setDefaultCommand(
                Commands.run(() -> m_flywheelSubsystem.setFlywheelVoltage(0.0), m_flywheelSubsystem));
        m_HoodSubsystem.setDefaultCommand(new SetHoodAngleCommand(m_HoodSubsystem, 74));
        m_indexSubsystem.setDefaultCommand(new SetIndexSpeedsCommand(m_indexSubsystem, 0.0, 0.0));

        m_controller.rightTrigger(0.8).or(m_controller.leftBumper()).whileTrue(prepareToShoot);
        m_controller.rightTrigger(0.8).and(readyToShoot).whileTrue(new ActivateAgitatedIndexCommand(m_indexSubsystem));
        readyToShootEntry
                .accept((m_flywheelSubsystem.flywheelAtTarget).and(m_HoodSubsystem.hoodAtTarget).getAsBoolean());
    }

    public Command getAutonomousCommand() {
        return null;
    }
}
