// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Consumer;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class PIDLogger {
    
    private final Consumer<double[]> onUpdate;
    private final DoubleEntry[] entries;
    
    public double[] values;

    public PIDLogger(String name, double[] defaults, Consumer<double[]> onUpdate) {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("Commands/" + name);
        String[] keys = {"kP", "kI", "kD", "vel", "accel", "kS", "kV", "kA", "kG"};
        
        this.entries = new DoubleEntry[keys.length];
        this.values = new double[keys.length];
        
        for (int i = 0; i < keys.length; i++) {
            entries[i] = createEntry(table, keys[i], defaults[i]);
        }
        
        this.onUpdate = onUpdate;
        update();
    }

    public void update() {
        for (int i = 0; i < entries.length; i++) {
            values[i] = entries[i].get();
        }
        
        onUpdate.accept(values);
    }

    private DoubleEntry createEntry(NetworkTable table, String key, double defaultValue) {
        DoubleEntry entry = table.getDoubleTopic(key).getEntry(defaultValue);
        entry.set(defaultValue);
        return entry;
    }
}
