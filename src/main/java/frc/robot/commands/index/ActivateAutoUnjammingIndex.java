// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.index;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.Constants;
import frc.robot.Constants.IndexConstants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ActivateAutoUnjammingIndex extends Command {

    private final IndexSubsystem indexSubsystem;
    private Timer timer;
    private Timer cooldownTimer;

    public static final double DEFAULT_HOPPER_UNJAM_THRESHOLD_CURRENT = 40.0;

    private double activatedSpeed;
    private double unjamCooldown;
    private double unjamThreshold;
    private double unjamSpeed;
    private double unjamDuration;
    private boolean isUnjamRoutineRunning;

    private final NetworkTable loggingTable = NetworkTableInstance.getDefault().getTable("Commands/" + getName());
    private DoublePublisher averagedCurrentPublisher;
    private DoubleEntry activatedSpeedEntry;
    private DoubleEntry unjamCooldownEntry;
    private DoubleEntry unjamThresholdEntry;
    private DoubleEntry unjamSpeedEntry;
    private DoubleEntry unjamDurationEntry;

    private double averagedHopperCurrent;
    private LinearFilter hopperCurrentFilter = LinearFilter.movingAverage(5);

    /** Creates a new ActivateMagazineCommand. */
    public ActivateAutoUnjammingIndex(IndexSubsystem indexSubsystem) {
        this.timer = new Timer();
        this.cooldownTimer = new Timer();
        this.indexSubsystem = indexSubsystem;

        averagedCurrentPublisher = loggingTable.getDoubleTopic("Average Hopper Current Amps").publish();
        activatedSpeedEntry = createEntry(loggingTable, "Fire Speed", IndexConstants.HOPPER_SPEED);
        unjamThresholdEntry = createEntry(loggingTable, "Unjam Threshold Current",
                DEFAULT_HOPPER_UNJAM_THRESHOLD_CURRENT);
        unjamSpeedEntry = createEntry(loggingTable, "Unjam Speed", -0.5);
        unjamDurationEntry = createEntry(loggingTable, "Unjam Duration Seconds", 0.25);
        unjamCooldownEntry = createEntry(loggingTable, "Unjam Cooldown Seconds", 0.5);

        unjamThreshold = DEFAULT_HOPPER_UNJAM_THRESHOLD_CURRENT;

        addRequirements(indexSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        isUnjamRoutineRunning = false;

        activatedSpeed = activatedSpeedEntry.get();
        unjamCooldown = unjamCooldownEntry.get();
        unjamSpeed = unjamSpeedEntry.get();
        unjamDuration = unjamDurationEntry.get();
        unjamThreshold = unjamThresholdEntry.get();

        hopperCurrentFilter.reset();

        timer.reset();
        timer.stop();
        cooldownTimer.reset();
        cooldownTimer.start();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        indexSubsystem.setMagazineSpeed(Constants.IndexConstants.MAGAZINE_SPEED);
        if (!isUnjamRoutineRunning)
            indexSubsystem.setHopperSpeed(activatedSpeed);
        else
            indexSubsystem.setHopperSpeed(unjamSpeed);

        averagedHopperCurrent = hopperCurrentFilter.calculate(indexSubsystem.getHopperCurrent());
        averagedCurrentPublisher.accept(averagedHopperCurrent);

        if (averagedHopperCurrent > unjamThreshold && !isUnjamRoutineRunning && cooldownTimer.get() > unjamCooldown) {
            isUnjamRoutineRunning = true;
            timer.reset();
            timer.start();
        }

        if (isUnjamRoutineRunning && timer.get() >= unjamDuration) {
            isUnjamRoutineRunning = false;
            timer.reset();
            timer.stop();
            cooldownTimer.reset();
            cooldownTimer.start();
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
