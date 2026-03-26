package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Inches;

public class FieldZoneManager {
    private RobotState robotState = RobotState.getInstance();
    // These positions are obtained through the FRC field dimensons
    // Each zone dimension holds a set of coordinates that goes from smaller to
    // greater. They are also either x coordinates or y coordinates.

    // Alliance areas (only the y coordinates because the alliance zones cover the
    // entire widths (x) of the field)
    private double[] blueAllianceZoneDimensions = { 0.0, Inches.of(182.11).in(Meters) };
    private double[] redAllianceZoneDimensions = { Inches.of(469.11).in(Meters), Inches.of(651.22).in(Meters) };

    // Middle zone (only the y coordinates in order to determine if the robot is in
    // the middle before determining if it's in depot or human player)
    private double[] middleZoneDimensions = { Inches.of(182.11).in(Meters), Inches.of(469.11).in(Meters) };

    // Human player and depot zones (taken from the prespective of the blue
    // alliance, only holds x coordinates)
    private double[] blueAllianceDepotZoneDimensions = { Inches.of(158.84).in(Meters), Inches.of(317.69).in(Meters) };
    private double[] blueAllianceHumanPlayerZoneDimensions = { 0.0, Inches.of(158.84).in(Meters) };

    // Trench Y coordinates (the range of the trench's y positions, contains two
    // possible ranges)
    private double[][] trenchesYPositions = { { Inches.of(158.61).in(Meters), Inches.of(205.61).in(Meters) },
            { Inches.of(445.61).in(Meters), Inches.of(492.61).in(Meters) } };

    // Trench X coordinates (the range of the trench's x positions, contains two
    // possible ranges)
    private double[][] trenchesXPositions = { { 0.0, Inches.of(49.94).in(Meters) },
            { Inches.of(267.75).in(Meters), Inches.of(317.69).in(Meters) } };

    private double bufferZone = 2; // Temp place holder value for the middle buffer zone

    private static FieldZoneManager instance = null;

    private boolean lastInDepot = false; // this helps the logic with the buffer zone

    public Trigger inTrench;

    public Trigger inAlliance;
    public Trigger inEnemy;

    public Trigger inDepot;
    public Trigger inHumanPlayer;

    private TriggerLogger triggerLogger = TriggerLogger.getInstance();

    private FieldZoneManager() {

        inTrench = new Trigger(() -> isRobotInTrench());
        inAlliance = new Trigger(() -> isRobotInAlliance());
        inEnemy = new Trigger(() -> isRobotInEnemy());
        inDepot = new Trigger(() -> isRobotInDepot());
        inHumanPlayer = new Trigger(() -> isRobotInHumanPlayer());

        triggerLogger.addTrigger(inTrench, "FieldZoneManager/inTrench");
        triggerLogger.addTrigger(inAlliance, "FieldZoneManager/inAlliance");
        triggerLogger.addTrigger(inEnemy, "FieldZoneManager/inEnemy");
        triggerLogger.addTrigger(inDepot, "FieldZoneManager/inDepot");
        triggerLogger.addTrigger(inHumanPlayer, "FieldZoneManager/inHumanPlayer");

    }

    public boolean isRobotInAlliance() {
        Pose2d estimatedTurretPose = robotState.getTurretOdomPose();

        if (DriverStation.getAlliance().get() == Alliance.Red) {
            if (estimatedTurretPose.getY() > redAllianceZoneDimensions[0]
                    && estimatedTurretPose.getY() < redAllianceZoneDimensions[1]) {
                return true;

            }
            return false;

        } else {
            if (estimatedTurretPose.getY() > blueAllianceZoneDimensions[0]
                    && estimatedTurretPose.getY() < blueAllianceZoneDimensions[1]) {
                return true;

            }
            return false;

        }
    }

    public boolean isRobotInEnemy() {
        Pose2d estimatedTurretPose = robotState.getTurretOdomPose();

        if (DriverStation.getAlliance().get() == Alliance.Red) {
            if (estimatedTurretPose.getY() > redAllianceZoneDimensions[0]
                    && estimatedTurretPose.getY() < redAllianceZoneDimensions[1]) {
                return false;

            }
            return true;

        } else {
            if (estimatedTurretPose.getY() > blueAllianceZoneDimensions[0]
                    && estimatedTurretPose.getY() < blueAllianceZoneDimensions[1]) {
                return false;

            }
            return true;

        }

    }

    public boolean isRobotInDepot() {
        Pose2d estimatedTurretPose = robotState.getTurretOdomPose();

        // if the robot is within the y coordinates of the middle zone
        if (estimatedTurretPose.getY() > middleZoneDimensions[0]
                && estimatedTurretPose.getY() < middleZoneDimensions[1]) {
            if (DriverStation.getAlliance().get() == Alliance.Blue) {

                // Checks if the robot is in the depot zone plus the buffered area.
                if (estimatedTurretPose.getX() > blueAllianceDepotZoneDimensions[0] - bufferZone / 2
                        && estimatedTurretPose.getX() < blueAllianceDepotZoneDimensions[1]) {
                    // Checks if the robot is in the core depot zone (without the buffered area).
                    if (estimatedTurretPose.getX() > blueAllianceDepotZoneDimensions[0] + bufferZone / 2
                            && estimatedTurretPose.getX() < blueAllianceDepotZoneDimensions[1]) {
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
                if (estimatedTurretPose.getX() > blueAllianceHumanPlayerZoneDimensions[0]
                        && estimatedTurretPose.getX() < blueAllianceHumanPlayerZoneDimensions[1] + bufferZone / 2) {
                    if (estimatedTurretPose.getX() > blueAllianceHumanPlayerZoneDimensions[0]
                            && estimatedTurretPose.getX() < blueAllianceHumanPlayerZoneDimensions[1] - bufferZone / 2) {
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
        lastInDepot = false;
        return false;

    }

    public boolean isRobotInHumanPlayer() {
        // Since all the areas are mutually exclusive, this is the logic.
        if (!isRobotInAlliance() && !isRobotInEnemy() && !isRobotInDepot()) {
            return true;
        }

        return false;

    }

    public boolean isRobotInTrench() {
        Pose2d estimatedTurretPose = robotState.getTurretOdomPose();

        // iterates through the two possible x ranges that the trench can be located in
        for (int i = 0; i < 2; i++) {
            if (estimatedTurretPose.getX() > trenchesXPositions[i][0]
                    && estimatedTurretPose.getX() < trenchesXPositions[i][1]) {
                // iterates through the two possible y ranges that the trench can be located in
                for (int j = 0; j < 2; j++) {
                    if (estimatedTurretPose.getY() > trenchesYPositions[i][0]
                            && estimatedTurretPose.getY() < trenchesYPositions[i][1]) {
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
