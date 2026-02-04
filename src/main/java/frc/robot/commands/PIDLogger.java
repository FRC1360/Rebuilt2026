// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Consumer;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class PIDLogger {

    class ClosedLoopConstants {
        public double kP;
        public double kI;
        public double kD;
        public double maxVelocity;
        public double maxAcceleration;
        public double kS;
        public double kV;
        public double kA;
        public double kG;

        public ClosedLoopConstants(double kP, double kI, double kD, double maxVelocity, double maxAcceleration, double kS, double kV, double kA, double kG) {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
            this.maxVelocity = maxVelocity;
            this.maxAcceleration = maxAcceleration;
            this.kS = kS;
            this.kV = kV;
            this.kA = kA;
            this.kG = kG;
        }
    }
    
    private final NetworkTable loggingTable;
    private final DoublePublisher controlLoopOutputPublisher;
    private final DoublePublisher setpointPositionPublisher;
    private final DoublePublisher setpointVelocityPublisher;
    private final DoublePublisher currentPositionPublisher;
    private final DoublePublisher currentVelocityPublisher;
    private final DoubleEntry kP_Entry;
    private final DoubleEntry kI_Entry;
    private final DoubleEntry kD_Entry;
    private final DoubleEntry maxAcceleration_Entry;
    private final DoubleEntry maxVelocity_Entry;
    private final DoubleEntry kS_Entry;
    private final DoubleEntry kV_Entry;
    private final DoubleEntry kA_Entry;
    private final DoubleEntry kG_Entry;
    
    private final Consumer<ClosedLoopConstants> pullConstantsConsumer;
    private final ClosedLoopConstants defaultConstants;

    public PIDLogger(String key, ClosedLoopConstants defaultConstants, Consumer<ClosedLoopConstants> pullConstantsConsumer) {
        this.pullConstantsConsumer = pullConstantsConsumer;
        this.defaultConstants = defaultConstants;

        loggingTable = NetworkTableInstance.getDefault().getTable(key);
        controlLoopOutputPublisher = loggingTable.getDoubleTopic("ControlLoopOutput").publish();
        setpointPositionPublisher = loggingTable.getDoubleTopic("SetpointPosition").publish();
        setpointVelocityPublisher = loggingTable.getDoubleTopic("SetpointVelocity").publish();
        currentPositionPublisher = loggingTable.getDoubleTopic("CurrentPosition").publish();
        currentVelocityPublisher = loggingTable.getDoubleTopic("CurrentVelocity").publish();

        kP_Entry = createEntry(loggingTable, "kP", this.defaultConstants.kP);
        kI_Entry = createEntry(loggingTable, "kI", this.defaultConstants.kI);
        kD_Entry = createEntry(loggingTable, "kD", this.defaultConstants.kD);
        maxVelocity_Entry = createEntry(loggingTable, "maxVelocity", this.defaultConstants.maxVelocity);
        maxAcceleration_Entry = createEntry(loggingTable, "maxAcceleration", this.defaultConstants.maxAcceleration);
        kS_Entry = createEntry(loggingTable, "kS", this.defaultConstants.kS);
        kV_Entry = createEntry(loggingTable, "kV", this.defaultConstants.kV);
        kA_Entry = createEntry(loggingTable, "kA", this.defaultConstants.kA);
        kG_Entry = createEntry(loggingTable, "kG", this.defaultConstants.kG);
    }

    public void updateConstants() {
        pullConstantsConsumer.accept(
            new ClosedLoopConstants(
                kP_Entry.get(),
                kI_Entry.get(),
                kD_Entry.get(),
                maxVelocity_Entry.get(),
                maxAcceleration_Entry.get(),
                kS_Entry.get(),
                kV_Entry.get(),
                kA_Entry.get(),
                kG_Entry.get()
            )
        );
    }

    public void logOutputs(double controlLoopOutput, double setpointPosition, double setpointVelocity, double currentPosition, double currentVelocity) {
        controlLoopOutputPublisher.set(controlLoopOutput);
        setpointPositionPublisher.set(setpointPosition);
        setpointVelocityPublisher.set(setpointVelocity);
        currentPositionPublisher.set(currentPosition);
        currentVelocityPublisher.set(currentVelocity);
    }

    private DoubleEntry createEntry(NetworkTable table, String key, double defaultValue) {
        DoubleEntry entry = table.getDoubleTopic(key).getEntry(defaultValue);
        entry.set(defaultValue);
        return entry;
    }
}
