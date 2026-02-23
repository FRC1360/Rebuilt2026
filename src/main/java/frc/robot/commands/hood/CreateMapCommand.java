// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.hood;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class CreateMapCommand {

    public static InterpolatingDoubleTreeMap turretPowerAngleDistancetable = new InterpolatingDoubleTreeMap();

    public CreateMapCommand() {
        turretPowerAngleDistancetable.put(1.0, 2.0); // meters and degrees
    }

}
