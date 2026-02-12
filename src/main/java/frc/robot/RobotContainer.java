// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.util.RobotState;

public class RobotContainer {

    //Replace with CommandPS4Controller or CommandJoystick if needed
    private final CommandXboxController m_controller = new CommandXboxController(0);
    private static final RobotState robotState = RobotState.getInstance();


    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        m_controller.x().onTrue(
            Commands.run(() -> robotState.getTurretOdomPose())
        ); 
    }

    public Command getAutonomousCommand() {
        return null;
    }
}
