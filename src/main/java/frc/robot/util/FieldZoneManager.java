package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Inches;

public class FieldZoneManager {
    private RobotState robotState = RobotState.getInstance();
    // These positions are obtained through the FRC field dimensons

    private double redTrenchCenterX = Inches.of(469.11).in(Meters);

    private double blueTrenchCenterX = Inches.of(182.11).in(Meters);

    private double trenchCenterY = Inches.of(158.845).in(Meters);

    // Trench Y coordinates (the range of the trench's y positions, contains two
    // possible ranges)
    private double[][] trenchesXPositions = { { Inches.of(158.61).in(Meters), Inches.of(205.61).in(Meters) },
            { Inches.of(445.61).in(Meters), Inches.of(492.61).in(Meters) } };

    // Trench X coordinates (the range of the trench's x positions, contains two
    // possible ranges)
    private double[][] trenchesYPositions = { { 0.0, Inches.of(49.94).in(Meters) },
            { Inches.of(267.75).in(Meters), Inches.of(317.69).in(Meters) } };

    private double centerAxisBuffer = 2; 
    private double middleZoneBuffer = 2; 

    private static FieldZoneManager instance = null;

    private boolean lastInDepot = false; // this helps the logic with the buffer zone
    private boolean lastInHumanPlayer = false; // this helps the logic with the buffer zone
    private boolean lastInEnemy = false;
    private boolean lastInAlliance = false;
    private boolean lastInMiddle = false;

    public Trigger inTrench;

    public Trigger inAlliance;
    public Trigger inEnemy;
    public Trigger inMiddle;

    public Trigger inDepot;
    public Trigger inHumanPlayer;

    private TriggerLogger triggerLogger = TriggerLogger.getInstance();

    private FieldZoneManager() {

        inTrench = new Trigger(() -> isRobotInTrench());
        inAlliance = new Trigger(() -> isRobotInAlliance());
        inEnemy = new Trigger(() -> isRobotInEnemy());
        inDepot = new Trigger(() -> isRobotInDepot());
        inMiddle = new Trigger(() -> isRobotInMiddle());
        inHumanPlayer = new Trigger(() -> isRobotInHumanPlayer());

        triggerLogger.addTrigger(inTrench, "FieldZoneManager/inTrench");
        triggerLogger.addTrigger(inAlliance, "FieldZoneManager/inAlliance");
        triggerLogger.addTrigger(inEnemy, "FieldZoneManager/inEnemy");
        triggerLogger.addTrigger(inDepot, "FieldZoneManager/inDepot");
        triggerLogger.addTrigger(inHumanPlayer, "FieldZoneManager/inHumanPlayer");
        triggerLogger.addTrigger(inMiddle, "FieldZoneManager/inMiddle");


    }

    public boolean isRobotInAlliance() {
        Pose2d estimatedTurretPose = robotState.getTurretOdomPose();

        if (!RobotState.getInstance().isBlueAlliance.getAsBoolean()) {
            if (estimatedTurretPose.getX() > redTrenchCenterX - middleZoneBuffer / 2) {
                if (estimatedTurretPose.getX() > redTrenchCenterX + middleZoneBuffer / 2) {
                    lastInAlliance = true;
                    return true;

                }

                if (lastInAlliance){
                    return true;
                }
                
                return false;
            }
            lastInAlliance = false;
            return false;

        } else {
             if (estimatedTurretPose.getX() < blueTrenchCenterX + middleZoneBuffer / 2) {
                if(estimatedTurretPose.getX() < blueTrenchCenterX - middleZoneBuffer / 2){
                    lastInAlliance = true;
                    return true;
                }

                if (lastInAlliance){
                    return true;
                }

                return false;
            }

            lastInAlliance = false;
            return false;

        }
    }

    public boolean isRobotInEnemy() {
        Pose2d estimatedTurretPose = robotState.getTurretOdomPose();

        if (!RobotState.getInstance().isBlueAlliance.getAsBoolean()) {
            if (estimatedTurretPose.getX() < blueTrenchCenterX + middleZoneBuffer / 2) {
                if(estimatedTurretPose.getX() < blueTrenchCenterX - middleZoneBuffer / 2){
                    lastInEnemy = true;
                    return true;

                }

                 if (lastInEnemy ){
                    return true;

                }
                
                return false;
            }

            lastInEnemy = false;
            return false;
            

        
            

        } else {
            if (estimatedTurretPose.getX() > redTrenchCenterX - middleZoneBuffer / 2) {
                if (estimatedTurretPose.getX() > redTrenchCenterX +  middleZoneBuffer / 2) {
                    lastInEnemy = true;
                    return true;
                }

                if (lastInEnemy){
                    return true;
                }

                return false;
            }

            lastInEnemy = false;
            return false;

        }

    }

    public boolean isRobotInMiddle(){
        Pose2d estimatedTurretPose = robotState.getTurretOdomPose();

        if(estimatedTurretPose.getX() > blueTrenchCenterX - middleZoneBuffer / 2 && estimatedTurretPose.getX() < redTrenchCenterX + middleZoneBuffer / 2){
            if(estimatedTurretPose.getX() > blueTrenchCenterX + middleZoneBuffer / 2 && estimatedTurretPose.getX() < redTrenchCenterX - middleZoneBuffer / 2){
                lastInMiddle = true;
                return true;
            }

            if (lastInMiddle){
                return true;
            }

            return false;
        }
        lastInMiddle = false;
        return false;

    }

    public boolean isRobotInDepot() {
        Pose2d estimatedTurretPose = robotState.getTurretOdomPose();

        // if the robot is within the y coordinates of the middle zone
     
        if (RobotState.getInstance().isBlueAlliance.getAsBoolean()) {
            // Checks if the robot is in the depot zone plus the buffered area.
            if (estimatedTurretPose.getY() > trenchCenterY - centerAxisBuffer / 2) {
                // Checks if the robot is in the core depot zone (without the buffered area).
                if (estimatedTurretPose.getY() > trenchCenterY + centerAxisBuffer / 2) {
                    lastInDepot = true;
                    return true;

                }

                if (lastInDepot) {
                    return true;

                }

                return false;

            }
            lastInDepot = false;
            return false;

        } else {
            if (estimatedTurretPose.getY() < trenchCenterY + centerAxisBuffer / 2) {
                if (estimatedTurretPose.getY() < trenchCenterY - centerAxisBuffer / 2) {
                    lastInDepot = true;
                    return true;
                }

                if (lastInDepot) {
                    return true;
                }
                return false;

            }
            lastInDepot = false;
            return false;
        }


    }

    public boolean isRobotInHumanPlayer() {
        // Since all the areas are mutually exclusive, this is the logic.
        Pose2d estimatedTurretPose = robotState.getTurretOdomPose();

        // if the robot is within the y coordinates of the middle zone
     
        if (RobotState.getInstance().isBlueAlliance.getAsBoolean()) {
             if (estimatedTurretPose.getY() < trenchCenterY + centerAxisBuffer / 2) {
                if (estimatedTurretPose.getY() < trenchCenterY - centerAxisBuffer / 2) {
                    lastInHumanPlayer = true;
                    return true;

                }

                if (lastInHumanPlayer) {
                    return true;

                }

                return false;

            }
            lastInHumanPlayer = false;
            return false;

        } else {
            if (estimatedTurretPose.getY() > trenchCenterY - centerAxisBuffer / 2) {
                if (estimatedTurretPose.getY() > trenchCenterY + centerAxisBuffer / 2) {
                    lastInHumanPlayer = true;
                    return true;
                }

                if (lastInHumanPlayer) {
                    return true;
                }
                return false;

            }
            lastInHumanPlayer = false;
            return false;
        }

    }

    public boolean isRobotInTrench() {
        Pose2d estimatedTurretPose = robotState.getTurretOdomPose();

        //iterates through the two possible x ranges that the trench can be located in
        for (int i = 0; i < 2; i++) {
            if (estimatedTurretPose.getX() > trenchesXPositions[i][0]
                    && estimatedTurretPose.getX() < trenchesXPositions[i][1]) {
                // iterates through the two possible y ranges that the trench can be located in
                for (int j = 0; j < 2; j++) {
                    if (estimatedTurretPose.getY() > trenchesYPositions[j][0]
                            && estimatedTurretPose.getY() < trenchesYPositions[j][1]) {
                        // basically if the robot satisfies both conditions.
                        return true;
                    }
                }

            }
        }

        return false;
    }

    public static synchronized FieldZoneManager getInstance() {
        if (instance == null) {
            instance = new FieldZoneManager();
        }

        return instance;
    }

}
