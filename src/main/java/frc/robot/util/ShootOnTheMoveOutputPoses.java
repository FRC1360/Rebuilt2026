package frc.robot.util;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.ShootingConstants;

public class ShootOnTheMoveOutputPoses {
    private Translation2d goalPose = new Translation2d();
    private Translation2d[] iteratedPoses = new Translation2d[ShootingConstants.MAX_ITERATION_COUNT];

    public ShootOnTheMoveOutputPoses() {
        
    }

    public void setGoalTranslation(Translation2d goalToSet) {
        this.goalPose = goalToSet;
    }
    public Translation2d getGoalTranslation() {
        return this.goalPose;
    }

    public void setIteratedTranslationWithIndex(int index, Translation2d poseToSet) {
        iteratedPoses[index] = poseToSet;
    }
    public Translation2d[] getIteratedTranslations() {
        return this.iteratedPoses;
    }
}

