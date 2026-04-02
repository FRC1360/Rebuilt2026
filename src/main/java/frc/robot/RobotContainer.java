// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.SetShooterFromCompensatedPoseCommand;
import frc.robot.commands.TrenchRunCommands;
import frc.robot.commands.flywheel.SetFlywheelVelocityCommand;
import frc.robot.commands.flywheel.SetFlywheelVelocityFromNetworkTables;
import frc.robot.commands.flywheel.SetFlywheelVelocityFromPoseCommand;
import frc.robot.commands.hood.SetHoodAngleCommand;
import frc.robot.commands.hood.SetHoodAngleFromNetworkTables;
import frc.robot.commands.hood.SetHoodAngleFromPose;
import frc.robot.commands.index.ActivateAgitatedIndexCommand;
import frc.robot.commands.index.ActivateAutoUnjammingIndex;
import frc.robot.commands.index.SetIndexSpeedsCommand;
import frc.robot.commands.intake.DeployIntakeCommand;
import frc.robot.commands.intake.RetractIntakeCommand;
import frc.robot.commands.intake.SetIntakePivotAngleCommand;
import frc.robot.commands.turret.AimTurretAtPoseCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.util.FieldConstants;
import frc.robot.util.FieldZoneManager;
import frc.robot.util.RobotState;
import frc.robot.util.TriggerLogger;

public class RobotContainer {

    private final RobotState robotState = RobotState.getInstance();
    private final FieldZoneManager m_fieldZoneManager = FieldZoneManager.getInstance();
    private final TriggerLogger triggerLogger = TriggerLogger.getInstance();

    private final CommandXboxController m_controller = new CommandXboxController(0);
    private final CommandXboxController m_operatorController = new CommandXboxController(1);
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
    private final IndexSubsystem m_indexSubsystem = new IndexSubsystem();
    private final FlywheelSubsystem m_flywheelSubsystem = new FlywheelSubsystem();
    private final HoodSubsystem m_HoodSubsystem = new HoodSubsystem();
    private final TurretSubsystem m_TurretSubsystem = new TurretSubsystem();
    private final Timer turretTimer = new Timer();
    private final Timer hoodTimer = new Timer();
    private final Timer flywheelTimer = new Timer();

    // Slow down speed when intaking and/or shooting
    private static final double SLOW_DRIVE_TRANSLATIONAL_MULTIPLIER = 0.3;
    private static final double SLOW_DRIVE_ANGULAR_MULTIPLIER = 0.8;

    private final SwerveTelemetry swerveLogger = new SwerveTelemetry(DriveCommands.MAX_DRIVE_TRANSLATIONAL_SPEED);

    public SendableChooser<Command> autoChooser;
    private PathPlannerAuto leftSideBasicAuto;
    private PathPlannerAuto rightSideBasicAuto;
    private PathPlannerAuto middleBasicAuto;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        robotState.setAllSuppliers(
                () -> drivetrain.getState().Pose,
                () -> drivetrain.getState().Speeds,
                () -> m_TurretSubsystem.getCurrentRobotRelativeRotation());

        configureAllAutos();

        configureBindings();
        drivetrain.registerTelemetry(swerveLogger::telemeterize);

        // Run warmup command for pathplanner as per CTRE example
        CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());

        SmartDashboard.putNumber("turret time to pose", turretTimer.get());
        SmartDashboard.putNumber("hood time to pose", hoodTimer.get());
        SmartDashboard.putNumber("flywheel time to pose", flywheelTimer.get());
    }

    private void configureAllAutos() {

        Trigger shooterAtSetpoints = m_flywheelSubsystem.flywheelAtTarget.and(m_HoodSubsystem.hoodAtTarget);
        Command runShotSequence = Commands.either(
                Commands.sequence(
                        new SetFlywheelVelocityFromPoseCommand(m_flywheelSubsystem,
                                FieldConstants.BLUE_ALLIANCE_HUB_POSE)
                                .alongWith(new SetHoodAngleFromPose(m_HoodSubsystem,
                                        FieldConstants.BLUE_ALLIANCE_HUB_POSE))
                                .until(shooterAtSetpoints).withTimeout(3.0),
                        new SetFlywheelVelocityFromPoseCommand(m_flywheelSubsystem,
                                FieldConstants.BLUE_ALLIANCE_HUB_POSE)
                                .alongWith(new SetHoodAngleFromPose(m_HoodSubsystem,
                                        FieldConstants.BLUE_ALLIANCE_HUB_POSE))
                                .alongWith(new ActivateAgitatedIndexCommand(m_indexSubsystem))
                                .withTimeout(3.0),
                        new SetFlywheelVelocityFromPoseCommand(m_flywheelSubsystem,
                                FieldConstants.BLUE_ALLIANCE_HUB_POSE)
                                .alongWith(new SetHoodAngleFromPose(m_HoodSubsystem,
                                        FieldConstants.BLUE_ALLIANCE_HUB_POSE))
                                .alongWith(new ActivateAgitatedIndexCommand(m_indexSubsystem))
                                .alongWith(new SetIntakePivotAngleCommand(m_intakeSubsystem, 45, 0.0))
                                .withTimeout(1.5),
                        new SetFlywheelVelocityFromPoseCommand(m_flywheelSubsystem,
                                FieldConstants.BLUE_ALLIANCE_HUB_POSE)
                                .alongWith(new SetHoodAngleFromPose(m_HoodSubsystem,
                                        FieldConstants.BLUE_ALLIANCE_HUB_POSE))
                                .alongWith(new ActivateAgitatedIndexCommand(m_indexSubsystem))
                                .alongWith(new RetractIntakeCommand(m_intakeSubsystem))
                                .withTimeout(3.0)),
                Commands.sequence(
                        new SetFlywheelVelocityFromPoseCommand(m_flywheelSubsystem,
                                FieldConstants.RED_ALLIANCE_HUB_POSE)
                                .alongWith(new SetHoodAngleFromPose(m_HoodSubsystem,
                                        FieldConstants.RED_ALLIANCE_HUB_POSE))
                                .until(shooterAtSetpoints).withTimeout(3.0),
                        new SetFlywheelVelocityFromPoseCommand(m_flywheelSubsystem,
                                FieldConstants.RED_ALLIANCE_HUB_POSE)
                                .alongWith(new SetHoodAngleFromPose(m_HoodSubsystem,
                                        FieldConstants.RED_ALLIANCE_HUB_POSE))
                                .alongWith(new ActivateAgitatedIndexCommand(m_indexSubsystem))
                                .withTimeout(3.0),
                        new SetFlywheelVelocityFromPoseCommand(m_flywheelSubsystem,
                                FieldConstants.RED_ALLIANCE_HUB_POSE)
                                .alongWith(new SetHoodAngleFromPose(m_HoodSubsystem,
                                        FieldConstants.RED_ALLIANCE_HUB_POSE))
                                .alongWith(new ActivateAgitatedIndexCommand(m_indexSubsystem))
                                .alongWith(new SetIntakePivotAngleCommand(m_intakeSubsystem, 45, 0.0))
                                .withTimeout(1.5),
                        new SetFlywheelVelocityFromPoseCommand(m_flywheelSubsystem,
                                FieldConstants.RED_ALLIANCE_HUB_POSE)
                                .alongWith(new SetHoodAngleFromPose(m_HoodSubsystem,
                                        FieldConstants.RED_ALLIANCE_HUB_POSE))
                                .alongWith(new ActivateAgitatedIndexCommand(m_indexSubsystem))
                                .alongWith(new RetractIntakeCommand(m_intakeSubsystem))
                                .withTimeout(3.0)),
                robotState.isBlueAlliance);

        /* Configure stuff for right side */
        rightSideBasicAuto = new PathPlannerAuto("Right Auto");
        rightSideBasicAuto.event("DEPLOY_AND_RUN_INTAKE_TRIGGER")
                .onTrue(new DeployIntakeCommand(m_intakeSubsystem, () -> true));
        rightSideBasicAuto.event("DEPLOY_INTAKE_STOP_ROLLERS_TRIGGER")
                .onTrue(new DeployIntakeCommand(m_intakeSubsystem, () -> false));
        rightSideBasicAuto.event("EXECUTE_SHOT_ROUTINE_COMMAND").onTrue(runShotSequence);

        /* Configure stuff for left side */
        leftSideBasicAuto = new PathPlannerAuto("Left Auto");
        leftSideBasicAuto.event("DEPLOY_AND_RUN_INTAKE_TRIGGER")
                .onTrue(new DeployIntakeCommand(m_intakeSubsystem, () -> true));
        leftSideBasicAuto.event("DEPLOY_INTAKE_STOP_ROLLERS_TRIGGER")
                .onTrue(new DeployIntakeCommand(m_intakeSubsystem, () -> false));
        leftSideBasicAuto.event("EXECUTE_SHOT_ROUTINE_COMMAND").onTrue(runShotSequence);

        middleBasicAuto = new PathPlannerAuto("Center Auto");
        middleBasicAuto.event("DEPLOY_AND_RUN_INTAKE_TRIGGER")
                .onTrue(new DeployIntakeCommand(m_intakeSubsystem, () -> true));
        middleBasicAuto.event("DEPLOY_INTAKE_STOP_ROLLERS_TRIGGER")
                .onTrue(new DeployIntakeCommand(m_intakeSubsystem, () -> false));
        middleBasicAuto.event("EXECUTE_SHOT_ROUTINE_COMMAND").onTrue(runShotSequence);

        autoChooser = new SendableChooser<Command>();
        autoChooser.setDefaultOption("Do Nothing", Commands.none());
        autoChooser.addOption("Left Side Neutral Zone", leftSideBasicAuto);
        autoChooser.addOption("Right Side Neutral Zone", rightSideBasicAuto);
        autoChooser.addOption("Center Fire Preload", middleBasicAuto);
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    private void configureBindings() {
        /*
         * Triger Configuration For Inputs and Setpoints
         */
        Trigger backdriveIntakeInput = m_controller.povLeft();
        Trigger backdriveIndexInput = m_controller.povUp();
        Trigger backdriveShooterInput = m_controller.povRight();
        Trigger backdrivingAnySubsystem = backdriveIntakeInput
                .or(backdriveIndexInput)
                .or(backdriveShooterInput);

        Trigger shootingInput = m_controller.rightBumper().and(backdrivingAnySubsystem.negate());
        Trigger shootingWithTurretInput = m_controller.rightTrigger(0.8).and(backdrivingAnySubsystem.negate());
        Trigger shootingThroughNetworkTablesInput = m_controller.povDown().and(backdrivingAnySubsystem.negate());
        Trigger passingInput = m_controller.leftBumper().and(backdrivingAnySubsystem.negate());
        Trigger intakePivotInput = m_controller.a();
        Trigger intakeRollerInput = m_controller.leftTrigger(0.8).and(backdriveIndexInput.negate());
        Trigger intakeAgitateInput = m_controller.b();
        Trigger trenchRun = m_controller.y();

        Trigger runIndexOverrideInput = m_controller.x();
        Trigger automaticShootCondition = (shootingInput.or(passingInput).or(shootingWithTurretInput)
                .or(shootingThroughNetworkTablesInput))
                .and(m_flywheelSubsystem.flywheelAtTarget)
                .and(m_HoodSubsystem.hoodAtTarget)
                .and((m_TurretSubsystem.turretAtTarget
                        .and(shootingInput.or(shootingWithTurretInput).or(shootingThroughNetworkTablesInput)))
                        .or(m_TurretSubsystem.turretAtPassingTarget.and(passingInput)));
        Trigger preparedAndReadyToShoot = backdrivingAnySubsystem.negate()
                .and(automaticShootCondition.or(runIndexOverrideInput));

        triggerLogger.addTrigger(preparedAndReadyToShoot, "RobotContainer/Ready To Shoot");
        triggerLogger.addTrigger(m_TurretSubsystem.turretAtTarget, "RobotContainer/Turret At Setpoint");
        triggerLogger.addTrigger(m_HoodSubsystem.hoodAtTarget, "RobotContainer/Hood At Setpoint");
        triggerLogger.addTrigger(m_flywheelSubsystem.flywheelAtTarget, "RobotContainer/Flywheel At Setpoint");
        triggerLogger.addTrigger(robotState.isBlueAlliance, "RobotContainer/Is Blue Alliance");

        // Driving
        Command joystickDriveAtNormalSpeed = DriveCommands.joystickDriveCommand(
                drivetrain, m_controller,
                1.0, 1.0);
        Command joystickDriveAtSlowSpeed = DriveCommands.joystickDriveCommand(
                drivetrain, m_controller,
                SLOW_DRIVE_TRANSLATIONAL_MULTIPLIER, SLOW_DRIVE_ANGULAR_MULTIPLIER);
        Command joystickDriveWhileFacingHub = Commands.either(
                DriveCommands.joystickDriveFacingPoseCommand(drivetrain, m_controller,
                        SLOW_DRIVE_TRANSLATIONAL_MULTIPLIER, FieldConstants.BLUE_ALLIANCE_HUB_POSE,
                        Rotation2d.fromDegrees(-90.0))
                        .until(robotState.isBlueAlliance.negate()),
                DriveCommands.joystickDriveFacingPoseCommand(drivetrain, m_controller,
                        SLOW_DRIVE_TRANSLATIONAL_MULTIPLIER, FieldConstants.RED_ALLIANCE_HUB_POSE,
                        Rotation2d.fromDegrees(90.0))
                        .until(robotState.isBlueAlliance),
                robotState.isBlueAlliance);
        Command joystickDriveWhilePassing = Commands.either(
                Commands.either(
                        DriveCommands.joystickDriveFacingPoseCommand(drivetrain, m_controller,
                                1.0, FieldConstants.BLUE_HUMAN_SIDE_PASS_POSE,
                                Rotation2d.fromDegrees(-90.0)),
                        DriveCommands.joystickDriveFacingPoseCommand(drivetrain, m_controller,
                                1.0, FieldConstants.BLUE_DEPOT_SIDE_PASS_POSE,
                                Rotation2d.fromDegrees(-90.0)),
                        () -> robotState.getTurretOdomPose().getY() < FieldConstants.BLUE_ALLIANCE_HUB_POSE.getY()),
                Commands.either(
                        DriveCommands.joystickDriveFacingPoseCommand(drivetrain, m_controller,
                                1.0, FieldConstants.RED_HUMAN_SIDE_PASS_POSE,
                                Rotation2d.fromDegrees(90.0)),
                        DriveCommands.joystickDriveFacingPoseCommand(drivetrain, m_controller,
                                1.0, FieldConstants.RED_DEPOT_SIDE_PASS_POSE,
                                Rotation2d.fromDegrees(90.0)),
                        () -> robotState.getTurretOdomPose().getY() > FieldConstants.RED_ALLIANCE_HUB_POSE.getY()),
                robotState.isBlueAlliance);

        trenchRun.whileTrue(TrenchRunCommands.autoTrenchRunCommand(drivetrain));

        drivetrain.setDefaultCommand(joystickDriveAtNormalSpeed);
        shootingWithTurretInput.whileTrue(joystickDriveAtSlowSpeed);
        shootingInput.whileTrue(joystickDriveWhileFacingHub);
        // passingInput.whileTrue(joystickDriveWhilePassing);

        // Intaking
        Command setIntakePivotBasedOnState = Commands.either(
                new DeployIntakeCommand(m_intakeSubsystem, intakeRollerInput)
                        .until(robotState.isIntakeCurrentlyDeployed.negate()),
                new RetractIntakeCommand(m_intakeSubsystem)
                        .until(robotState.isIntakeCurrentlyDeployed),
                robotState.isIntakeCurrentlyDeployed).repeatedly();

        Command agitateIntake = Commands.repeatingSequence(
                new SetIntakePivotAngleCommand(m_intakeSubsystem, 55.0, 0.1)
                        .withTimeout(0.5),
                new SetIntakePivotAngleCommand(m_intakeSubsystem, 5.0, 0.1)
                        .withTimeout(0.5));

        m_intakeSubsystem.setDefaultCommand(setIntakePivotBasedOnState);
        intakePivotInput.onTrue(robotState.toggleIntakeState);
        intakeAgitateInput.whileTrue(agitateIntake);

        // Shooting
        Command prepareToShootAtHub = Commands.either(
                Commands.parallel(
                        new SetHoodAngleFromPose(m_HoodSubsystem,
                                FieldConstants.BLUE_ALLIANCE_HUB_POSE),
                        new SetFlywheelVelocityFromPoseCommand(m_flywheelSubsystem,
                                FieldConstants.BLUE_ALLIANCE_HUB_POSE)),
                Commands.parallel(
                        new SetHoodAngleFromPose(m_HoodSubsystem,
                                FieldConstants.RED_ALLIANCE_HUB_POSE),
                        new SetFlywheelVelocityFromPoseCommand(m_flywheelSubsystem,
                                FieldConstants.RED_ALLIANCE_HUB_POSE)),
                robotState.isBlueAlliance);
        Command prepareToPass = Commands.either(
                Commands.either(
                        new SetShooterFromCompensatedPoseCommand(m_TurretSubsystem, m_HoodSubsystem,
                                m_flywheelSubsystem, FieldConstants.BLUE_HUMAN_SIDE_PASS_POSE),
                        new SetShooterFromCompensatedPoseCommand(m_TurretSubsystem, m_HoodSubsystem,
                                m_flywheelSubsystem, FieldConstants.BLUE_DEPOT_SIDE_PASS_POSE),
                        () -> robotState.getTurretOdomPose().getY() < FieldConstants.BLUE_ALLIANCE_HUB_POSE.getY()),
                Commands.either(
                        new SetShooterFromCompensatedPoseCommand(m_TurretSubsystem, m_HoodSubsystem,
                                m_flywheelSubsystem, FieldConstants.RED_HUMAN_SIDE_PASS_POSE),
                        new SetShooterFromCompensatedPoseCommand(m_TurretSubsystem, m_HoodSubsystem,
                                m_flywheelSubsystem, FieldConstants.RED_DEPOT_SIDE_PASS_POSE),
                        () -> robotState.getTurretOdomPose().getY() > FieldConstants.RED_ALLIANCE_HUB_POSE.getY()),
                robotState.isBlueAlliance);
        Command shootFromNetworkTables = Commands.either(
                Commands.parallel(
                        new SetHoodAngleFromNetworkTables(m_HoodSubsystem,
                                FieldConstants.BLUE_ALLIANCE_HUB_POSE),
                        new SetFlywheelVelocityFromNetworkTables(m_flywheelSubsystem)),
                Commands.parallel(
                        new SetHoodAngleFromNetworkTables(m_HoodSubsystem,
                                FieldConstants.RED_ALLIANCE_HUB_POSE),
                        new SetFlywheelVelocityFromNetworkTables(m_flywheelSubsystem)),
                robotState.isBlueAlliance);

        m_flywheelSubsystem.setDefaultCommand(new SetFlywheelVelocityCommand(m_flywheelSubsystem,
                10.0));
        m_HoodSubsystem.setDefaultCommand(new SetHoodAngleCommand(m_HoodSubsystem,
                74));
        m_indexSubsystem.setDefaultCommand(new SetIndexSpeedsCommand(m_indexSubsystem,
                0.0, 0.3));
        // m_TurretSubsystem.setDefaultCommand(new
        // SetTurretToNonWrappedEncoderCommand(m_TurretSubsystem,
        // TurretConstants.ENCODER_STARTUP_ANGLE_DEGREES));
        m_TurretSubsystem.setDefaultCommand(Commands.either(
                new AimTurretAtPoseCommand(m_TurretSubsystem, FieldConstants.BLUE_ALLIANCE_HUB_POSE)
                        .until(robotState.isBlueAlliance.negate()),
                new AimTurretAtPoseCommand(m_TurretSubsystem, FieldConstants.RED_ALLIANCE_HUB_POSE)
                        .until(robotState.isBlueAlliance),
                robotState.isBlueAlliance));

        shootingInput.whileTrue(prepareToShootAtHub);
        passingInput.whileTrue(prepareToPass);
        preparedAndReadyToShoot.whileTrue(new ActivateAutoUnjammingIndex(m_indexSubsystem));
        shootingWithTurretInput.whileTrue(Commands.either(
                new SetShooterFromCompensatedPoseCommand(m_TurretSubsystem, m_HoodSubsystem, m_flywheelSubsystem,
                        FieldConstants.BLUE_ALLIANCE_HUB_POSE),
                new SetShooterFromCompensatedPoseCommand(m_TurretSubsystem, m_HoodSubsystem, m_flywheelSubsystem,
                        FieldConstants.RED_ALLIANCE_HUB_POSE),
                robotState.isBlueAlliance)
                .alongWith(
                        new RunCommand(() -> {
                            turretTimer.start();
                            if (m_TurretSubsystem.turretAtTarget.getAsBoolean() == true) {
                                turretTimer.stop();
                            }
                        }))
                .alongWith(
                        new RunCommand(() -> {
                            hoodTimer.start();
                            if (m_HoodSubsystem.hoodAtTarget.getAsBoolean() == true) {
                                hoodTimer.stop();
                            }
                        }))
                .alongWith(
                        new RunCommand(() -> {
                            flywheelTimer.start();
                            if (m_flywheelSubsystem.flywheelAtTarget.getAsBoolean() == true) {
                                flywheelTimer.stop();
                            }
                        })));
        shootingThroughNetworkTablesInput.whileTrue(Commands.parallel(
                new SetHoodAngleFromNetworkTables(m_HoodSubsystem, FieldConstants.RED_ALLIANCE_HUB_POSE),
                new SetFlywheelVelocityFromNetworkTables(m_flywheelSubsystem),
                new AimTurretAtPoseCommand(m_TurretSubsystem, FieldConstants.RED_ALLIANCE_HUB_POSE)));

        // Backdriving
        Command backdriveIntakeWhileKeepingState = Commands.either(
                new SetIntakePivotAngleCommand(m_intakeSubsystem, IntakeConstants.PIVOT_DEPLOYED_ANGLE, -0.5)
                        .until(robotState.isIntakeCurrentlyDeployed.negate()),
                new SetIntakePivotAngleCommand(m_intakeSubsystem, IntakeConstants.PIVOT_RETRACTED_ANGLE, -0.5)
                        .until(robotState.isIntakeCurrentlyDeployed),
                robotState.isIntakeCurrentlyDeployed).repeatedly();
        backdriveIntakeInput.whileTrue(backdriveIntakeWhileKeepingState);

        Command backdriveIndex = new SetIndexSpeedsCommand(m_indexSubsystem, -0.4, -1.0);
        backdriveIndexInput.whileTrue(backdriveIndex);

        Command backdriveShooter = new SetFlywheelVelocityCommand(m_flywheelSubsystem, -30.0)
                .alongWith(new SetIndexSpeedsCommand(m_indexSubsystem, -0.4, -1.0));
        backdriveShooterInput.whileTrue(backdriveShooter);

        /* Overall Fudge Factor */
        m_operatorController.rightBumper().whileTrue(robotState.incrementFlywheelFudgeFactor);
        m_operatorController.leftBumper().whileTrue(robotState.decrementFlywheelFudgeFactor);
        m_operatorController.a().whileTrue(robotState.resetFlywheelSpeedFudgeFactor);

        m_operatorController.rightTrigger(0.8).whileTrue(robotState.incrementHoodFudgeFactor);
        m_operatorController.leftTrigger(0.8).whileTrue(robotState.decrementHoodFudgeFactor);
        m_operatorController.y().whileTrue(robotState.resetHoodFudgeFactor);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
