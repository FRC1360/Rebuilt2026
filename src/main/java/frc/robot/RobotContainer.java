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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.flywheel.SetFlywheelVelocityFromPoseCommand;
import frc.robot.commands.hood.SetHoodAngleCommand;
import frc.robot.commands.hood.SetHoodAngleFromPose;
import frc.robot.commands.index.ActivateAgitatedIndexCommand;
import frc.robot.commands.index.SetIndexSpeedsCommand;
import frc.robot.commands.intake.DeployIntakeCommand;
import frc.robot.commands.intake.RetractIntakeCommand;
import frc.robot.commands.intake.SetIntakePivotAngleCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.util.FieldConstants;
import frc.robot.util.RobotState;

public class RobotContainer {

    private final RobotState robotState = RobotState.getInstance();

    // Replace with CommandPS4Controller or CommandJoystick if needed
    private final CommandXboxController m_controller = new CommandXboxController(0);
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
    private final IndexSubsystem m_indexSubsystem = new IndexSubsystem();
    private final FlywheelSubsystem m_flywheelSubsystem = new FlywheelSubsystem();
    private final HoodSubsystem m_HoodSubsystem = new HoodSubsystem();
    private final TurretSubsystem m_turretSubsysem = new TurretSubsystem();

    // Use theoretical 'max' speed for normal driving & 3/4ths rotations per sec.
    private static final double NORMAL_DRIVE_TRANSLATIONAL_SPEED = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private static final double NORMAL_DRIVE_ANGULAR_SPEED = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    // Slow down speed when intaking and/or shooting
    private static final double SLOW_DRIVE_TRANSLATIONAL_SPEED = 0.3 * NORMAL_DRIVE_TRANSLATIONAL_SPEED;
    private static final double SLOW_DRIVE_ANGULAR_SPEED = 1.0 * NORMAL_DRIVE_ANGULAR_SPEED;

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric normalTeleopDriveRequest = new SwerveRequest.FieldCentric()
            .withDeadband(NORMAL_DRIVE_TRANSLATIONAL_SPEED * 0.15)
            .withRotationalDeadband(NORMAL_DRIVE_ANGULAR_SPEED * 0.15)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.FieldCentric slowTeleopDriveRequest = new SwerveRequest.FieldCentric()
            .withDeadband(SLOW_DRIVE_TRANSLATIONAL_SPEED * 0.15)
            .withRotationalDeadband(SLOW_DRIVE_ANGULAR_SPEED * 0.15)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.FieldCentricFacingAngle driveFacingAngle = new SwerveRequest.FieldCentricFacingAngle()
            .withDeadband(SLOW_DRIVE_TRANSLATIONAL_SPEED * 0.15)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withHeadingPID(5.0, 0.0, 0.0);

    private final SwerveTelemetry swerveLogger = new SwerveTelemetry(NORMAL_DRIVE_TRANSLATIONAL_SPEED);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        robotState.setAllSuppliers(
                () -> drivetrain.getState().Pose,
                () -> m_turretSubsysem.getCurrentRobotRelativeRotation());

        configureBindings();
        drivetrain.registerTelemetry(swerveLogger::telemeterize);

        // Run warmup command for pathplanner as per CTRE example
        CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());
    }

    private void configureBindings() {
        /*
         * Triger Configuration For Inputs and Setpoints
         */
        Trigger shootingInput = m_controller.rightTrigger(0.8);
        Trigger intakePivotInput = m_controller.a();
        Trigger intakeRollerInput = m_controller.leftTrigger(0.8);
        Trigger intakeAgitateInput = m_controller.b();

        Trigger preparedAndReadyToShoot = shootingInput
                .and(m_flywheelSubsystem.flywheelAtTarget)
                .and(m_HoodSubsystem.hoodAtTarget);

        Trigger slowDriveModeActivated = shootingInput;

        // Driving
        Command joystickDriveAtNormalSpeed = drivetrain.applyRequest(
                () -> normalTeleopDriveRequest
                        .withVelocityX(-m_controller.getLeftY() * NORMAL_DRIVE_TRANSLATIONAL_SPEED)
                        .withVelocityY(-m_controller.getLeftX() * NORMAL_DRIVE_TRANSLATIONAL_SPEED)
                        .withRotationalRate(-m_controller.getRightX() * NORMAL_DRIVE_ANGULAR_SPEED));
        Command joystickDriveAtSlowSpeed = drivetrain.applyRequest(
                () -> slowTeleopDriveRequest
                        .withVelocityX(-m_controller.getLeftY() * SLOW_DRIVE_TRANSLATIONAL_SPEED)
                        .withVelocityY(-m_controller.getLeftX() * SLOW_DRIVE_TRANSLATIONAL_SPEED)
                        .withRotationalRate(-m_controller.getRightX() * SLOW_DRIVE_ANGULAR_SPEED));
        Command joystickDriveWhileFacingHub = drivetrain.applyRequest(
                () -> driveFacingAngle
                        .withVelocityX(-m_controller.getLeftY() * SLOW_DRIVE_TRANSLATIONAL_SPEED)
                        .withVelocityY(-m_controller.getLeftX() * SLOW_DRIVE_TRANSLATIONAL_SPEED)
                        .withTargetDirection(PhotonUtils.getYawToPose(
                                new Pose2d(
                                        robotState.getTurretOdomPose().getTranslation(),
                                        new Rotation2d()),
                                FieldConstants.RED_ALLIANCE_HUB_POSE).plus(Rotation2d.fromDegrees(90.0))));

        drivetrain.setDefaultCommand(joystickDriveAtNormalSpeed);
        slowDriveModeActivated.whileTrue(joystickDriveWhileFacingHub);
        m_controller.y().onTrue(Commands.runOnce(() -> drivetrain.seedFieldCentric(), drivetrain));

        // Intaking
        Command setIntakePivotBasedOnState = Commands.either(
                new DeployIntakeCommand(m_intakeSubsystem, intakeRollerInput)
                        .until(robotState.isIntakeCurrentlyDeployed.negate()),
                new RetractIntakeCommand(m_intakeSubsystem)
                        .until(robotState.isIntakeCurrentlyDeployed),
                robotState.isIntakeCurrentlyDeployed).repeatedly();

        Command agitateIntake = Commands.repeatingSequence(
                new SetIntakePivotAngleCommand(m_intakeSubsystem, 45.0, () -> true)
                        .withTimeout(0.75),
                new SetIntakePivotAngleCommand(m_intakeSubsystem, 5.0, () -> true)
                        .withTimeout(0.75));

        m_intakeSubsystem.setDefaultCommand(setIntakePivotBasedOnState);
        intakePivotInput.onTrue(robotState.toggleIntakeState);
        intakeAgitateInput.whileTrue(agitateIntake);

        // Shooting
        Command disableFlywheels = Commands.run(() -> m_flywheelSubsystem.setFlywheelVoltage(0.0), m_flywheelSubsystem);

        Command prepareToShoot = new SetFlywheelVelocityFromPoseCommand(m_flywheelSubsystem,
                FieldConstants.RED_ALLIANCE_HUB_POSE)
                .alongWith(new SetHoodAngleFromPose(m_HoodSubsystem, FieldConstants.RED_ALLIANCE_HUB_POSE));

        m_flywheelSubsystem.setDefaultCommand(disableFlywheels);
        m_HoodSubsystem.setDefaultCommand(new SetHoodAngleCommand(m_HoodSubsystem, 74));
        m_turretSubsysem.setDefaultCommand(Commands.run(() -> m_turretSubsysem.setVoltage(0.0), m_turretSubsysem));
        m_indexSubsystem.setDefaultCommand(new SetIndexSpeedsCommand(m_indexSubsystem, 0.0, 0.0));

        shootingInput.whileTrue(prepareToShoot);
        preparedAndReadyToShoot.whileTrue(new ActivateAgitatedIndexCommand(m_indexSubsystem));

        // SysID
        m_controller.povUp().whileTrue(m_turretSubsysem.sysIdQuasistatic(Direction.kForward));
        m_controller.povDown().whileTrue(m_turretSubsysem.sysIdQuasistatic(Direction.kReverse));
        m_controller.povRight().whileTrue(m_turretSubsysem.sysIdDynamic(Direction.kForward));
        m_controller.povLeft().whileTrue(m_turretSubsysem.sysIdDynamic(Direction.kReverse));
    }

    public Command getAutonomousCommand() {
        return null;
    }
}
