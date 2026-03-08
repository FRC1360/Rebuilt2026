// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.photonvision.PhotonUtils;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.flywheel.SetFlywheelVelocityCommand;
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
import frc.robot.util.FieldConstants;
import frc.robot.util.RobotState;

public class RobotContainer {

    private final RobotState robotState = RobotState.getInstance();

    private final CommandXboxController m_controller = new CommandXboxController(0);
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
    private final IndexSubsystem m_indexSubsystem = new IndexSubsystem();
    private final FlywheelSubsystem m_flywheelSubsystem = new FlywheelSubsystem();
    private final HoodSubsystem m_HoodSubsystem = new HoodSubsystem();

    // Slow down speed when intaking and/or shooting
    private static final double SLOW_DRIVE_TRANSLATIONAL_MULTIPLIER = 0.3;
    private static final double SLOW_DRIVE_ANGULAR_MULTIPLIER = 1.0;

    private final SwerveTelemetry swerveLogger = new SwerveTelemetry(DriveCommands.MAX_DRIVE_TRANSLATIONAL_SPEED);

    private PathPlannerAuto testAuto1;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        robotState.setAllSuppliers(
                () -> drivetrain.getState().Pose,
                () -> Rotation2d.fromDegrees(90));

        configureAllAutos();

        configureBindings();
        drivetrain.registerTelemetry(swerveLogger::telemeterize);

        // Run warmup command for pathplanner as per CTRE example
        CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());
    }

    private void configureAllAutos() {
        testAuto1 = new PathPlannerAuto("Left Auto");
        testAuto1.event("L Deploy Intake").onTrue(new DeployIntakeCommand(m_intakeSubsystem, () -> true));
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

        // Driving
        Command joystickDriveAtNormalSpeed = DriveCommands.joystickDriveCommand(
                drivetrain, m_controller,
                1.0, 1.0);
        Command joystickDriveAtSlowSpeed = DriveCommands.joystickDriveCommand(
                drivetrain, m_controller,
                SLOW_DRIVE_TRANSLATIONAL_MULTIPLIER, SLOW_DRIVE_ANGULAR_MULTIPLIER);
        Command joystickDriveWhileFacingHub = Commands.either(
                DriveCommands.joystickDriveFacingAngleCommand(drivetrain, m_controller,
                        SLOW_DRIVE_TRANSLATIONAL_MULTIPLIER, PhotonUtils.getYawToPose(
                                new Pose2d(robotState.getTurretOdomPose().getTranslation(), new Rotation2d()),
                                FieldConstants.BLUE_ALLIANCE_HUB_POSE).plus(Rotation2d.fromDegrees(-90.0))),
                DriveCommands.joystickDriveFacingAngleCommand(drivetrain, m_controller,
                        SLOW_DRIVE_TRANSLATIONAL_MULTIPLIER, PhotonUtils.getYawToPose(
                                new Pose2d(robotState.getTurretOdomPose().getTranslation(), new Rotation2d()),
                                FieldConstants.RED_ALLIANCE_HUB_POSE).plus(Rotation2d.fromDegrees(90.0))),
                robotState.isBlueAlliance);

        drivetrain.setDefaultCommand(joystickDriveAtNormalSpeed);
        intakeRollerInput.and(shootingInput.negate()).whileTrue(joystickDriveAtSlowSpeed);
        shootingInput.whileTrue(joystickDriveWhileFacingHub);

        // Intaking
        Command setIntakePivotBasedOnState = Commands.either(
                new DeployIntakeCommand(m_intakeSubsystem, intakeRollerInput)
                        .until(robotState.isIntakeCurrentlyDeployed.negate()),
                new RetractIntakeCommand(m_intakeSubsystem)
                        .until(robotState.isIntakeCurrentlyDeployed),
                robotState.isIntakeCurrentlyDeployed).repeatedly();

        Command agitateIntake = Commands.repeatingSequence(
                new SetIntakePivotAngleCommand(m_intakeSubsystem, 45.0, -0.3)
                        .withTimeout(0.75),
                new SetIntakePivotAngleCommand(m_intakeSubsystem, 5.0, -0.3)
                        .withTimeout(0.75));

        m_intakeSubsystem.setDefaultCommand(setIntakePivotBasedOnState);
        intakePivotInput.onTrue(robotState.toggleIntakeState);
        intakeAgitateInput.whileTrue(agitateIntake);

        // Shooting
        Command prepareToShoot = Commands.either(
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

        m_flywheelSubsystem.setDefaultCommand(new SetFlywheelVelocityCommand(m_flywheelSubsystem, 10.0));
        m_HoodSubsystem.setDefaultCommand(new SetHoodAngleCommand(m_HoodSubsystem, 74));
        m_indexSubsystem.setDefaultCommand(new SetIndexSpeedsCommand(m_indexSubsystem, 0.0, 0.3));

        shootingInput.whileTrue(prepareToShoot);
        preparedAndReadyToShoot.whileTrue(new ActivateAgitatedIndexCommand(m_indexSubsystem));
    }

    public Command getAutonomousCommand() {
        return testAuto1;
    }
}
