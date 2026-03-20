package frc.robot.util;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.RobotState;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Inches;

public class FieldZoneManager {
    private RobotState robotState = RobotState.getInstance();
    // These positions are obtained through the FRC field dimensons
    
    private double[] blueAllianceZoneDimensions = {0.0, Inches.of(182.11).in(Meters)};
    private double[] redAllianceZoneDimensions = {Inches.of(469.11).in(Meters), Inches.of(651.22).in(Meters)};

    private double[] middleZoneDimensions = {Inches.of(182.11).in(Meters), Inches.of(469.11).in(Meters)};

    private double[] depotZoneDimensions = {Inches.of(158.84).in(Meters), Inches.of(317.69).in(Meters)};
    private double[] humanPlayerZoneDimensions = {0.0, Inches.of(158.84).in(Meters)};

    private double[][] trenchesYPositions = {{Inches.of(158.61).in(Meters), Inches.of(205.61).in(Meters)}, {Inches.of(445.61).in(Meters), Inches.of(492.61).in(Meters)}};
    private double[][] trenchesXPositions = {{0.0, Inches.of(49.94).in(Meters)}, {Inches.of(267.75).in(Meters), Inches.of(317.69).in(Meters)}};


    private double bufferZone = 6; // Temp place holder value

    private static FieldZoneManager instance = null;

    public Trigger inTrench;

    public Trigger inAlliance;
    public Trigger inEnemy;

    public Trigger inDepot;
    public Trigger inHumanPlayer;


    private FieldZoneManager(){

        inTrench = new Trigger(() -> isRobotInTrench());
        inAlliance = new Trigger(() -> isRobotInAlliance());
        inEnemy = new Trigger(() -> isRobotInEnemy());
        inDepot = new Trigger(() -> isRobotInDepot());
        inHumanPlayer = new Trigger(() -> isRobotInHumanPlayer());
        

    }

    public boolean isRobotInAlliance(){
        Pose2d estimatedTurretPose = robotState.getTurretOdomPose();

        if (DriverStation.getAlliance().get() == Alliance.Red){
            if (estimatedTurretPose.getY() > redAllianceZoneDimensions[0] && estimatedTurretPose.getY() < redAllianceZoneDimensions[1]){
                return true;

            }
            return false;

        } 
        else{
            if (estimatedTurretPose.getY() > blueAllianceZoneDimensions[0] && estimatedTurretPose.getY() < blueAllianceZoneDimensions[1]){
                return true;

            }
            return false;

        }
    }

    public boolean isRobotInEnemy(){
        Pose2d estimatedTurretPose = robotState.getTurretOdomPose();

        if (DriverStation.getAlliance().get() == Alliance.Red){
            if (estimatedTurretPose.getY() > redAllianceZoneDimensions[0] && estimatedTurretPose.getY() < redAllianceZoneDimensions[1]){
                return false;

            }
            return true;

        }
        else{
            if (estimatedTurretPose.getY() > blueAllianceZoneDimensions[0] && estimatedTurretPose.getY() < blueAllianceZoneDimensions[1]){
                return false;

            }
            return true;

        }
        
    }

    public boolean isRobotInDepot() {
        Pose2d estimatedTurretPose = robotState.getTurretOdomPose();
       
        if (estimatedTurretPose.getY() > middleZoneDimensions[0] && estimatedTurretPose.getY() < middleZoneDimensions[1]){
            if (DriverStation.getAlliance().get() == Alliance.Blue){
                if (estimatedTurretPose.getX() > depotZoneDimensions[0] - bufferZone / 2 && estimatedTurretPose.getX() < depotZoneDimensions[1]){
                    return true;
                }
                return false;
            }
            else{
                if (estimatedTurretPose.getX() > humanPlayerZoneDimensions[0] && estimatedTurretPose.getX() < humanPlayerZoneDimensions[1] + bufferZone / 2){
                    return true;

                }
                return false;
            }

        }
        return false;

    }

    public boolean isRobotInHumanPlayer() {
        Pose2d estimatedTurretPose = robotState.getTurretOdomPose();
       
        if (estimatedTurretPose.getY() > middleZoneDimensions[0] && estimatedTurretPose.getY() < middleZoneDimensions[1]){
            if (DriverStation.getAlliance().get() == Alliance.Blue){
                if (estimatedTurretPose.getX() > humanPlayerZoneDimensions[0] && estimatedTurretPose.getX() < humanPlayerZoneDimensions[1] + bufferZone / 2){
                    return true;
                }
                return false;
            }
            else{
                if (estimatedTurretPose.getX() > depotZoneDimensions[0] - bufferZone / 2 && estimatedTurretPose.getX() < depotZoneDimensions[1]){
                    return true;

                }
                return false;
            }

        }
        return false;

    }

    public boolean isRobotInTrench(){
        Pose2d estimatedTurretPose = robotState.getTurretOdomPose();

        for (int i = 0; i < 2; i++){
            if (estimatedTurretPose.getX() > trenchesXPositions[i][0] && estimatedTurretPose.getX() < trenchesXPositions[i][1]){
                for (int j = 0; j < 2; j++){
                    if (estimatedTurretPose.getY() > trenchesYPositions[i][0] && estimatedTurretPose.getY() < trenchesYPositions[i][1]){
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
