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
public class AgitationIndexCommand extends Command {

    private final IndexSubsystem indexSubsystem;

    private double intervalOneLength;
    private double intervalOneSpeed;
    private double intervalTwoLength;
    private double intervalTwoSpeed;

    private double intervalThreeLength;
    private double intervalThreeSpeed;
    private double intervalFourLength;
    private double intervalFourSpeed;

    private Timer timer;

    private final NetworkTable loggingTable = NetworkTableInstance.getDefault().getTable("Subsystems/Jiggle Index");

    private DoubleEntry intervalOnePeriodEntry;
    private DoubleEntry intervalOneSpeedEntry;
    private DoubleEntry intervalTwoPeriodEntry;
    private DoubleEntry intervalTwoSpeedEntry;

    private DoubleEntry intervalThreePeriodEntry;
    private DoubleEntry intervalThreeSpeedEntry;
    private DoubleEntry intervalFourPeriodEntry;
    private DoubleEntry intervalFourSpeedEntry;

    private double firstIntervalPeriod;
    private double secondIntervalPeriod;
    private double thirdIntervalPeriod;
    private double fourthIntervalPeriod;

    /** Creates a new ActivateMagazineCommand. */
    public AgitationIndexCommand(IndexSubsystem indexSubsystem) {
        this.timer = new Timer();
        this.indexSubsystem = indexSubsystem;

        intervalOnePeriodEntry = createEntry(loggingTable, "FeedForward/kS", this.intervalOneLength);
        intervalOneSpeedEntry = createEntry(loggingTable, "FeedForward/kV", this.intervalOneSpeed);
        intervalTwoPeriodEntry = createEntry(loggingTable, "FeedForward/kA", this.intervalTwoLength);
        intervalTwoPeriodEntry = createEntry(loggingTable, "FeedForward/kG", this.intervalTwoSpeed);

        intervalThreePeriodEntry = createEntry(loggingTable, "FeedForward/kS", this.intervalThreeLength);
        intervalThreeSpeedEntry = createEntry(loggingTable, "FeedForward/kV", this.intervalThreeSpeed);
        intervalFourPeriodEntry = createEntry(loggingTable, "FeedForward/kA", this.intervalFourLength);
        intervalFourPeriodEntry = createEntry(loggingTable, "FeedForward/kG", this.intervalFourSpeed);

        firstIntervalPeriod = intervalOneLength;
        secondIntervalPeriod = firstIntervalPeriod + intervalTwoLength;
        thirdIntervalPeriod = secondIntervalPeriod + intervalThreeLength;
        fourthIntervalPeriod = thirdIntervalPeriod + intervalFourLength;

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
        //sets the index speed for interval one.
        if (timer.get() < firstIntervalPeriod) 
        {
            indexSubsystem.setHopperSpeed(intervalOneSpeed);

        } 

        //sets the index speed for interval two.
        else if (timer.get() > firstIntervalPeriod && timer.get() < secondIntervalPeriod) 
        {
            indexSubsystem.setHopperSpeed(intervalTwoSpeed);

      
        } 
        
        //sets the index speed for interval three.
        else if (timer.get() > secondIntervalPeriod && timer.get() < thirdIntervalPeriod) 
        {
            indexSubsystem.setHopperSpeed(intervalThreeSpeed);
        
        } 

        //sets the index speed for interval four.
        else if (timer.get() > thirdIntervalPeriod && timer.get() < fourthIntervalPeriod) 
        {
            indexSubsystem.setHopperSpeed(intervalFourSpeed);
        } 

         //resets the timer and restarts the loop process.
        else{
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
