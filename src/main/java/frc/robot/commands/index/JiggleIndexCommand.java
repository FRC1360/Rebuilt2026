// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.index;

import java.lang.annotation.Target;
import java.time.Period;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.Constants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class JiggleIndexCommand extends Command {

    private final IndexSubsystem indexSubsystem;
    private double lastRecordedInterval = 0.0;

    private double intervalOnePeriod;
    private double intervalOneSpeed;
    private double intervalTwoPeriod;
    private double intervalTwoSpeed;

    private boolean rotatingClockwise = false;
    private Timer timer;

    private final NetworkTable loggingTable = NetworkTableInstance.getDefault().getTable("Subsystems/Jiggle Index");

    private DoubleEntry intervalOnePeriodEntry;
    private DoubleEntry intervalOneSpeedEntry;
    private DoubleEntry intervalTwoPeriodEntry;
    private DoubleEntry intervalTwoSpeedEntry;

    /** Creates a new ActivateMagazineCommand. */
    public JiggleIndexCommand(IndexSubsystem indexSubsystem) {
        this.timer = new Timer();
        this.indexSubsystem = indexSubsystem;

        intervalOnePeriodEntry = createEntry(loggingTable, "FeedForward/kS", this.intervalOnePeriod);
        intervalOneSpeedEntry = createEntry(loggingTable, "FeedForward/kV", this.intervalOneSpeed);
        intervalTwoPeriodEntry = createEntry(loggingTable, "FeedForward/kA", this.intervalTwoPeriod);
        intervalTwoPeriodEntry = createEntry(loggingTable, "FeedForward/kG", this.intervalTwoSpeed);

        addRequirements(indexSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        timer.start();

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        //sets the index in the positive direction.
        if (timer.get() < intervalOnePeriod) {
            indexSubsystem.setHopperSpeed(intervalOneSpeed);

        //sets the index in the negative direction.
        } else if (timer.get() > intervalOnePeriod && timer.get() < intervalOnePeriod + intervalTwoPeriod) {
            indexSubsystem.setHopperSpeed(intervalTwoSpeed);

        //resets the timer and restarts the loop process.
        } else {
            timer.reset();
        }

    }

    private DoubleEntry createEntry(NetworkTable table, String key, double defaultValue) {
        DoubleEntry entry = table.getDoubleTopic(key).getEntry(defaultValue);
        entry.set(defaultValue);
        return entry;
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
