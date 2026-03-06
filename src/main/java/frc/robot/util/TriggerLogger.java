package frc.robot.util;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class TriggerLogger {
    
    private final NetworkTable loggingTable;
    private static TriggerLogger instance = null;
    private Trigger[] triggerList;
    private  BooleanPublisher[] triggerPublishers;
    private int totalTriggers = 0;


    private TriggerLogger(){
        triggerList = new Trigger[50];
        triggerPublishers = new  BooleanPublisher[50];

        loggingTable = NetworkTableInstance.getDefault().getTable("Trigger Logger");
    }

    public static synchronized TriggerLogger getInstance() {
        if (instance == null) {
            instance = new TriggerLogger();
        }

        return instance;
    }

    public void addTrigger(Trigger trigger, String key){
        triggerList[totalTriggers] = trigger;
        triggerPublishers[totalTriggers] = loggingTable.getBooleanTopic(key).publish();

        totalTriggers += 1;
    }

    public void updateAllPublishers(){
        for (int i = 0; i < totalTriggers; i++){
            triggerPublishers[i].accept(triggerList[i].getAsBoolean());
     
        }
    }

   

}
