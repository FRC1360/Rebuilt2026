// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.util.FieldConstants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AimTurretAtHub extends Command {

    private final Supplier<Rotation2d> robotRotationSupplier;
    private final TurretSubsystem turretSubsystem;

    private Rotation2d targetFieldRelativeTurretRotation;
    private Rotation2d targetRobotRelativeTurretRotation;
    private Translation2d turretTranslation;
    private Translation2d hubTranslation;

    /** Creates a new AimTurretAtHub. */
    public AimTurretAtHub(TurretSubsystem turretSubsystem, Supplier<Rotation2d> robotRotationSupplier) {
        this.turretSubsystem = turretSubsystem;
        this.robotRotationSupplier = robotRotationSupplier;

        this.hubTranslation = FieldConstants.redAllianceHubPose.getTranslation();

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(this.turretSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        turretTranslation = turretSubsystem.getEstimatedPose().getTranslation();
        targetFieldRelativeTurretRotation = Rotation2d.fromRadians(
            Math.asin(
                (turretTranslation.getX() - hubTranslation.getX()) 
                / 
                (turretTranslation.getY() - hubTranslation.getY())
            )
        );
        targetRobotRelativeTurretRotation = 
            targetFieldRelativeTurretRotation.minus(robotRotationSupplier.get());
        turretSubsystem.setVoltage(
            turretSubsystem.closedLoopCalculate(targetRobotRelativeTurretRotation)
        );
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
