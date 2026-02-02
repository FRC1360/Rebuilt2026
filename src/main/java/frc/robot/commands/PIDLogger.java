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
    private final NetworkTable table;
    private final Consumer<PIDLogger> resetController;
    
    /// PID constants
    private final DoubleEntry kP;
    private final DoubleEntry kI;
    private final DoubleEntry kD;
    
    /// Feedforward constants
    private final DoubleEntry kS;
    private final DoubleEntry kV;
    private final DoubleEntry kA;
    private final DoubleEntry kG;

    public PIDLogger(String commandName, double default_kP, double default_kI, double default_kD, 
                     double default_kS, double default_kV, double default_kA, double default_kG,
                     Consumer<PIDLogger> resetController) {
        this.table = NetworkTableInstance.getDefault().getTable("Commands/" + commandName);
        this.resetController = resetController;
        
        /// Initialize PID entries
        kP = createEntry("kP", default_kP);
        kI = createEntry("kI", default_kI);
        kD = createEntry("kD", default_kD);
        
        /// Initialize feedforward entries
        kS = createEntry("kS", default_kS);
        kV = createEntry("kV", default_kV);
        kA = createEntry("kA", default_kA);
        kG = createEntry("kG", default_kG);
    }

    /// Reset all controller gains from NetworkTables
    public void reset() {
        resetController.accept(this);
    }

    public double getKP() { return kP.get(); }
    public double getKI() { return kI.get(); }
    public double getKD() { return kD.get(); }
    public double getKS() { return kS.get(); }
    public double getKV() { return kV.get(); }
    public double getKA() { return kA.get(); }
    public double getKG() { return kG.get(); }

    public DoublePublisher createPublisher(String key) {
        return table.getDoubleTopic(key).publish();
    }

    public DoubleEntry createEntry(String key, double defaultValue) {
        DoubleEntry entry = table.getDoubleTopic(key).getEntry(defaultValue);
        entry.set(defaultValue);
        return entry;
    }
}
