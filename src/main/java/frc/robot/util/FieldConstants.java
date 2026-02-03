package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class FieldConstants {
    public static final Pose2d redAllianceHubPose = 
        new Pose2d(
            new Translation2d(16.54 - 4.625, 4.035),
            new Rotation2d()
        );
        
    public static final Pose2d blueAllianceHubPose = 
        new Pose2d(
            new Translation2d(4.625, 4.035),
            new Rotation2d()
        );
}
