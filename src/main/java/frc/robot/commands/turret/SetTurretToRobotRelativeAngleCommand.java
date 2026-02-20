// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.turret;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurretSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetTurretToRobotRelativeAngleCommand extends Command {

    private final TurretSubsystem turretSubsystem;
    private final Rotation2d targetRotation;

    /** Creates a new SetRobotRelativeTurretRotation. */
    public SetTurretToRobotRelativeAngleCommand(TurretSubsystem turretSubsystem, Rotation2d targetRotation) {
        this.turretSubsystem = turretSubsystem;
        this.targetRotation = targetRotation;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(this.turretSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        turretSubsystem.grabConstantsFromNetworkTables();
        turretSubsystem.resetPIDController();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        turretSubsystem.setVoltage(
            turretSubsystem.closedLoopCalculate(targetRotation)
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
