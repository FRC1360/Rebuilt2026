// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.util.FieldConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
 
  public static class TurretConstants {
    public static final int TurretMotorID = 10;
    public static final double velocityConversionFactor = 0; 
    public static final double positionConversionFactor = 0;
    public static final Pose2d FIELD_RELATIVE_BLUE_HUB_POS = new Pose2d(FieldConstants.blueAllianceHubPose.getX(), FieldConstants.blueAllianceHubPose.getY(), FieldConstants.blueAllianceHubPose.getRotation());
    public static final Pose2d FIELD_RELATIVE_RED_HUB_POS = new Pose2d(FieldConstants.redAllianceHubPose.getX(), FieldConstants.redAllianceHubPose.getY(), FieldConstants.redAllianceHubPose.getRotation());


    public static final double Max_v = 0.0;
    public static final double Max_a = 0.0;
    public static final double kP = 0.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kA = 0.0;
    public static final double kV = 0.0;
    public static final double kG = 0.0;
    
  }
}
