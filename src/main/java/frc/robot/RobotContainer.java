// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.SetShooterFromCompensatedPoseCommand;
import frc.robot.commands.SetShooterFromPassingCompensatedPoseCommand;
import frc.robot.commands.flywheel.SetFlywheelVelocityCommand;
import frc.robot.commands.flywheel.SetFlywheelVelocityFromNetworkTables;
import frc.robot.commands.flywheel.SetFlywheelVelocityFromPoseCommand;
import frc.robot.commands.hood.SetHoodAngleCommand;
import frc.robot.commands.hood.SetHoodAngleFromNetworkTables;
import frc.robot.commands.hood.SetHoodAngleFromPose;
import frc.robot.commands.index.ActivateAutoUnjammingIndex;
import frc.robot.commands.index.SetIndexSpeedsCommand;
import frc.robot.commands.intake.DeployIntakeCommand;
import frc.robot.commands.intake.RetractIntakeCommand;
import frc.robot.commands.intake.SetIntakePivotAngleCommand;
import frc.robot.commands.turret.AimTurretAtPoseCommand;
import frc.robot.commands.turret.SetTurretToNonWrappedEncoderCommand;
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

    // Slow down speed when intaking and/or shooting
    private static final double SLOW_DRIVE_TRANSLATIONAL_MULTIPLIER = 0.3;
    private static final double SLOW_DRIVE_ANGULAR_MULTIPLIER = 0.8;

    private final SwerveTelemetry swerveLogger = new SwerveTelemetry(DriveCommands.MAX_DRIVE_TRANSLATIONAL_SPEED);

    public SendableChooser<Command> autoChooser;
    private PathPlannerAuto leftSideTurretedAuto1;
    // private PathPlannerPath exitLeft;
    // private PathPlannerPath enterLeft;
    // private PathPlannerPath exitRight;
    // private PathPlannerPath enterRight;
    // private PathConstraints pathConstraints;

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
    }

    private void configureAllAutos() {

        // try {
        //     enterLeft = PathPlannerPath.fromPathFile("Enter Alliance Left");
        //     enterRight = PathPlannerPath.fromPathFile("Enter Alliance Right");
        //     exitLeft = PathPlannerPath.fromPathFile("Leave Alliance Left");
        //     exitRight = PathPlannerPath.fromPathFile("Leave Alliance Right");
        // } catch (FileVersionException e) {
        //     e.printStackTrace();
        // } catch (IOException e) {
        //     e.printStackTrace();
        // } catch (ParseException e) {
        //     e.printStackTrace();
        // }

        leftSideTurretedAuto1 = new PathPlannerAuto("L1");

        // pathConstraints = new PathConstraints(
                // 3.0,
                // 3.0,
                // Units.degreesToRadians(540),
                // Units.degreesToRadians(720));

        Trigger shooterAtSetpoints = m_flywheelSubsystem.flywheelAtTarget
                .and(m_HoodSubsystem.hoodAtTarget)
                .and(m_TurretSubsystem.turretAtTarget);

        Command prepareLeftSideTurretedShot = Commands.either(
                Commands.parallel(
                    new SetHoodAngleFromPose(m_HoodSubsystem, FieldConstants.BLUE_ALLIANCE_HUB_POSE),
                    new SetFlywheelVelocityFromPoseCommand(m_flywheelSubsystem, FieldConstants.BLUE_ALLIANCE_HUB_POSE),
                    new SetTurretToNonWrappedEncoderCommand(m_TurretSubsystem, -195.0)
                ),
                Commands.parallel(
                    new SetHoodAngleFromPose(m_HoodSubsystem, FieldConstants.RED_ALLIANCE_HUB_POSE),
                    new SetFlywheelVelocityFromPoseCommand(m_flywheelSubsystem, FieldConstants.RED_ALLIANCE_HUB_POSE),
                    new SetTurretToNonWrappedEncoderCommand(m_TurretSubsystem, -195.0)
                ),
                robotState.isBlueAlliance);
        Command prepareTurretedShot = Commands.either(
                new SetShooterFromCompensatedPoseCommand(m_TurretSubsystem, m_HoodSubsystem, m_flywheelSubsystem,
                        FieldConstants.BLUE_ALLIANCE_HUB_POSE),
                new SetShooterFromCompensatedPoseCommand(m_TurretSubsystem, m_HoodSubsystem, m_flywheelSubsystem,
                        FieldConstants.RED_ALLIANCE_HUB_POSE),
                robotState.isBlueAlliance);
        Command executeTurretedShot = Commands.either(
                new SetShooterFromCompensatedPoseCommand(m_TurretSubsystem, m_HoodSubsystem, m_flywheelSubsystem,
                        FieldConstants.BLUE_ALLIANCE_HUB_POSE),
                new SetShooterFromCompensatedPoseCommand(m_TurretSubsystem, m_HoodSubsystem, m_flywheelSubsystem,
                        FieldConstants.RED_ALLIANCE_HUB_POSE),
                robotState.isBlueAlliance)
                .alongWith(Commands.none().until(shooterAtSetpoints)
                        .andThen(new ActivateAutoUnjammingIndex(m_indexSubsystem)))
                .until(leftSideTurretedAuto1.event("STOP_TURRETED_SHOOTING"));

        /* Configure stuff for left side */
        leftSideTurretedAuto1.event("DEPLOY_INTAKE_RUN_ROLLERS").onTrue(
                new DeployIntakeCommand(m_intakeSubsystem, () -> true));
        leftSideTurretedAuto1.event("DEPLOY_INTAKE_STOP_ROLLERS").onTrue(
                new DeployIntakeCommand(m_intakeSubsystem, () -> false));
        leftSideTurretedAuto1.event("PREPARE_TO_SHOOT").onTrue(prepareLeftSideTurretedShot);
        leftSideTurretedAuto1.event("EXECUTE_TURRETED_SHOOTING").onTrue(executeTurretedShot);

        autoChooser = new SendableChooser<Command>();
        autoChooser.setDefaultOption("Do Nothing", Commands.none());
        autoChooser.addOption("Left Side Turreted 1", leftSideTurretedAuto1);
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

        Trigger generalShootingInput = m_controller.rightTrigger(0.8).and(backdrivingAnySubsystem.negate());
        Trigger shootingIntoHubWithTurretInput = generalShootingInput.and(m_fieldZoneManager.inAlliance);
        Trigger passingWithTurretInput = generalShootingInput
                .and(m_fieldZoneManager.inEnemy.or(m_fieldZoneManager.inMiddle));
        Trigger shootingThroughNetworkTablesInput = m_controller.povDown().and(backdrivingAnySubsystem.negate());

        Trigger intakePivotInput = m_controller.a();
        Trigger intakeRollerInput = m_controller.leftTrigger(0.8).and(backdriveIndexInput.negate());

        Trigger trenchRunInput = m_controller.y();

        Trigger runIndexOverrideInput = m_controller.x();
        Trigger automaticShootCondition = (generalShootingInput.or(shootingThroughNetworkTablesInput))
                .and(m_flywheelSubsystem.flywheelAtTarget)
                .and(m_HoodSubsystem.hoodAtTarget)
                .and((m_TurretSubsystem.turretAtTarget
                        .and(shootingIntoHubWithTurretInput.or(shootingThroughNetworkTablesInput)))
                        .or(m_TurretSubsystem.turretAtPassingTarget.and(passingWithTurretInput)));
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
        Command joystickDriveAtShootOnTheMoveSpeed = DriveCommands.joystickDriveCommand(
                drivetrain, m_controller,
                SLOW_DRIVE_TRANSLATIONAL_MULTIPLIER, SLOW_DRIVE_ANGULAR_MULTIPLIER);

        Trigger isTurnInputFacingRight = new Trigger(() -> m_controller.getRightX() > 0);
        Command joystickDriveWithRightSideSweep = DriveCommands.joystickDriveWithCenterOfRotationCommand(
                drivetrain, m_controller,
                1.0, 1.0, new Translation2d(0.275, -0.33));
        Command joystickDriveWithLeftSideSweep = DriveCommands.joystickDriveWithCenterOfRotationCommand(
                drivetrain, m_controller,
                1.0, 1.0, new Translation2d(0.275, 0.33));
        Command joystickDriveWhileIntaking = Commands.either(
            joystickDriveWithRightSideSweep.until(isTurnInputFacingRight.negate()), 
            joystickDriveWithLeftSideSweep.until(isTurnInputFacingRight), 
            isTurnInputFacingRight).repeatedly();

        // Command trenchRunCommand = Commands.either(
        //         Commands.either(
        //                 AutoBuilder.pathfindThenFollowPath(exitRight, pathConstraints),
        //                 AutoBuilder.pathfindThenFollowPath(enterRight, pathConstraints),
        //                 m_fieldZoneManager.inAlliance).repeatedly(),
        //         Commands.either(
        //                 AutoBuilder.pathfindThenFollowPath(exitLeft, pathConstraints),
        //                 AutoBuilder.pathfindThenFollowPath(enterLeft, pathConstraints),
        //                 m_fieldZoneManager.inAlliance).repeatedly(),
        //         m_fieldZoneManager.inHumanPlayer).repeatedly();
        // trenchRun.whileTrue(trenchRunCommand);

        drivetrain.setDefaultCommand(joystickDriveAtNormalSpeed);
        shootingIntoHubWithTurretInput.whileTrue(joystickDriveAtShootOnTheMoveSpeed);
        intakeRollerInput.whileTrue(joystickDriveWhileIntaking);

        // Intaking
        Command setIntakePivotBasedOnState = Commands.either(
                new DeployIntakeCommand(m_intakeSubsystem, intakeRollerInput)
                        .until(robotState.isIntakeCurrentlyDeployed.negate()),
                new RetractIntakeCommand(m_intakeSubsystem)
                        .until(robotState.isIntakeCurrentlyDeployed),
                robotState.isIntakeCurrentlyDeployed).repeatedly();

        m_intakeSubsystem.setDefaultCommand(setIntakePivotBasedOnState);
        intakePivotInput.onTrue(robotState.toggleIntakeState);

        // Shooting
        Command autoAimTurretAtAllianceHub = Commands.either(
                new AimTurretAtPoseCommand(m_TurretSubsystem, FieldConstants.BLUE_ALLIANCE_HUB_POSE)
                        .until(robotState.isBlueAlliance.negate()),
                new AimTurretAtPoseCommand(m_TurretSubsystem, FieldConstants.RED_ALLIANCE_HUB_POSE)
                        .until(robotState.isBlueAlliance),
                robotState.isBlueAlliance);
        Command prepareToShootAtHubWithTurret = Commands.either(
                new SetShooterFromCompensatedPoseCommand(m_TurretSubsystem, m_HoodSubsystem, m_flywheelSubsystem,
                        FieldConstants.BLUE_ALLIANCE_HUB_POSE),
                new SetShooterFromCompensatedPoseCommand(m_TurretSubsystem, m_HoodSubsystem, m_flywheelSubsystem,
                        FieldConstants.RED_ALLIANCE_HUB_POSE),
                robotState.isBlueAlliance);
        Command prepareToPassWithTurret = Commands.either(
                Commands.either(
                        new SetShooterFromPassingCompensatedPoseCommand(m_TurretSubsystem, m_HoodSubsystem,
                                m_flywheelSubsystem, FieldConstants.BLUE_HUMAN_SIDE_PASS_POSE)
                                .until(m_fieldZoneManager.inDepot),
                        new SetShooterFromPassingCompensatedPoseCommand(m_TurretSubsystem, m_HoodSubsystem,
                                m_flywheelSubsystem, FieldConstants.BLUE_DEPOT_SIDE_PASS_POSE)
                                .until(m_fieldZoneManager.inHumanPlayer),
                        m_fieldZoneManager.inHumanPlayer).repeatedly(),
                Commands.either(
                        new SetShooterFromPassingCompensatedPoseCommand(m_TurretSubsystem, m_HoodSubsystem,
                                m_flywheelSubsystem, FieldConstants.RED_HUMAN_SIDE_PASS_POSE)
                                .until(m_fieldZoneManager.inDepot),
                        new SetShooterFromPassingCompensatedPoseCommand(m_TurretSubsystem, m_HoodSubsystem,
                                m_flywheelSubsystem, FieldConstants.RED_DEPOT_SIDE_PASS_POSE)
                                .until(m_fieldZoneManager.inHumanPlayer),
                        m_fieldZoneManager.inHumanPlayer).repeatedly(),
                robotState.isBlueAlliance);
        Command prepareToShootFromNetworktablesWithTurret = Commands.parallel(
                new SetHoodAngleFromNetworkTables(m_HoodSubsystem, FieldConstants.BLUE_DEPOT_SIDE_PASS_POSE),
                new SetFlywheelVelocityFromNetworkTables(m_flywheelSubsystem),
                new AimTurretAtPoseCommand(m_TurretSubsystem, FieldConstants.BLUE_DEPOT_SIDE_PASS_POSE));

        m_flywheelSubsystem.setDefaultCommand(
                new SetFlywheelVelocityCommand(m_flywheelSubsystem, 10.0));
        m_HoodSubsystem.setDefaultCommand(
                new SetHoodAngleCommand(m_HoodSubsystem, 74));
        m_indexSubsystem.setDefaultCommand(
                new SetIndexSpeedsCommand(m_indexSubsystem, 0.0, 0.3));
        m_TurretSubsystem.setDefaultCommand(autoAimTurretAtAllianceHub);

        shootingIntoHubWithTurretInput.whileTrue(prepareToShootAtHubWithTurret);
        passingWithTurretInput.whileTrue(prepareToPassWithTurret);
        shootingThroughNetworkTablesInput.whileTrue(prepareToShootFromNetworktablesWithTurret);
        preparedAndReadyToShoot.whileTrue(new ActivateAutoUnjammingIndex(m_indexSubsystem));

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
