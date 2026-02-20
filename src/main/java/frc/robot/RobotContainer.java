// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.turret.AimTurretAtPoseCommand;
import frc.robot.commands.turret.GetCurrentRobotRelativeRotationCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.util.FieldConstants;
import frc.robot.util.RobotState;

public class RobotContainer {

    //Replace with CommandPS4Controller or CommandJoystick if needed
    private final CommandXboxController m_controller = new CommandXboxController(0);
    private static final RobotState robotState = RobotState.getInstance();

    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.15).withRotationalDeadband(MaxAngularRate * 0.15) // Add a 15% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final Telemetry logger = new Telemetry(MaxSpeed);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final TurretSubsystem m_turretSubsystem = new TurretSubsystem();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        robotState.setAllSuppliers(
            () -> drivetrain.samplePoseAt(Timer.getFPGATimestamp()).get(),
            () -> m_turretSubsystem.getCurrentRotation(),
            () -> m_turretSubsystem.getPhotonCameraEstimatedPose(),
            () -> m_turretSubsystem.getPhotonCameraEstimatedPoseTimestamp()
        );

        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-m_controller.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-m_controller.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-m_controller.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );
        m_turretSubsystem.setDefaultCommand(
            new GetCurrentRobotRelativeRotationCommand(m_turretSubsystem, new Rotation2d())
        );

        m_controller.leftBumper().whileTrue(
            new AimTurretAtPoseCommand(m_turretSubsystem, FieldConstants.BLUE_ALLIANCE_HUB_POSE)
        );
    }

    public Command getAutonomousCommand() {
        return null;
    }
}
