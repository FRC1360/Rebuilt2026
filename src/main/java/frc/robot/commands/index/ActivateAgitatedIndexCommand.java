// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.index;

import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.Constants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ActivateAgitatedIndexCommand extends Command {

    private final IndexSubsystem indexSubsystem;
    private Timer timer;

    private double intervalOneDuration;
    private double intervalOneSpeed;
    private double intervalTwoDuration;
    private double intervalTwoSpeed;
    private double intervalThreeDuration;
    private double intervalThreeSpeed;
    private double intervalFourDuration;
    private double intervalFourSpeed;

    private final NetworkTable loggingTable = NetworkTableInstance.getDefault().getTable("Commands/" + getName());
    private DoubleEntry intervalOneDurationEntry;
    private DoubleEntry intervalOneSpeedEntry;
    private DoubleEntry intervalTwoDurationEntry;
    private DoubleEntry intervalTwoSpeedEntry;
    private DoubleEntry intervalThreeDurationEntry;
    private DoubleEntry intervalThreeSpeedEntry;
    private DoubleEntry intervalFourDurationEntry;
    private DoubleEntry intervalFourSpeedEntry;

    private double firstIntervalPeriod;
    private double secondIntervalPeriod;
    private double thirdIntervalPeriod;
    private double fourthIntervalPeriod;

    /** Creates a new ActivateMagazineCommand. */
    public ActivateAgitatedIndexCommand(IndexSubsystem indexSubsystem) {
        this.timer = new Timer();
        this.indexSubsystem = indexSubsystem;

        intervalOneDurationEntry = createEntry(loggingTable, "Interval 1", this.intervalOneDuration);
        intervalOneSpeedEntry = createEntry(loggingTable, "Interval 1", this.intervalOneSpeed);
        intervalTwoDurationEntry = createEntry(loggingTable, "Interval 1", this.intervalTwoDuration);
        intervalTwoSpeedEntry = createEntry(loggingTable, "Interval 1", this.intervalTwoSpeed);

        intervalThreeDurationEntry = createEntry(loggingTable, "Interval 1", this.intervalThreeDuration);
        intervalThreeSpeedEntry = createEntry(loggingTable, "Interval 1", this.intervalThreeSpeed);
        intervalFourDurationEntry = createEntry(loggingTable, "Interval 1", this.intervalFourDuration);
        intervalFourSpeedEntry = createEntry(loggingTable, "Interval 1", this.intervalFourSpeed);

        addRequirements(indexSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        intervalOneDuration = intervalOneDurationEntry.get();
        intervalOneSpeed = intervalOneSpeedEntry.get();

        intervalTwoDuration = intervalTwoDurationEntry.get();
        intervalTwoSpeed = intervalTwoSpeedEntry.get();

        intervalThreeDuration = intervalThreeDurationEntry.get();
        intervalThreeSpeed = intervalThreeSpeedEntry.get();

        intervalFourDuration = intervalFourDurationEntry.get();
        intervalFourSpeed = intervalFourSpeedEntry.get();

        firstIntervalPeriod = intervalOneDuration;
        secondIntervalPeriod = firstIntervalPeriod + intervalTwoDuration;
        thirdIntervalPeriod = secondIntervalPeriod + intervalThreeDuration;
        fourthIntervalPeriod = thirdIntervalPeriod + intervalFourDuration;

        timer.reset();
        timer.start();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // sets the index speed for interval one.
        if (timer.get() < firstIntervalPeriod) {
            indexSubsystem.setHopperSpeed(intervalOneSpeed);
        }

        // sets the index speed for interval two.
        else if (timer.get() > firstIntervalPeriod && timer.get() < secondIntervalPeriod) {
            indexSubsystem.setHopperSpeed(intervalTwoSpeed);
        }

        // sets the index speed for interval three.
        else if (timer.get() > secondIntervalPeriod && timer.get() < thirdIntervalPeriod) {
            indexSubsystem.setHopperSpeed(intervalThreeSpeed);
        }

        // sets the index speed for interval four.
        else if (timer.get() > thirdIntervalPeriod && timer.get() < fourthIntervalPeriod) {
            indexSubsystem.setHopperSpeed(intervalFourSpeed);
        }

        // resets the timer and restarts the loop process.
        else {
            timer.reset();
        }

        indexSubsystem.setMagazineSpeed(Constants.IndexConstants.MAGAZINE_SPEED);
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
