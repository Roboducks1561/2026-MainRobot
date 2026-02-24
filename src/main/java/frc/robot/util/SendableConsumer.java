package frc.robot.util;

import java.util.ArrayList;
import java.util.function.Consumer;
import java.util.function.DoubleConsumer;

import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.util.mapleSim.Bootleg2026;

//TODO, as of now only the individual creator works, fix that!!!
public class SendableConsumer {

    public static DoubleEntry createSendableChooser(String id, double initVal){
        NetworkTable table = NetworkTableInstance.getDefault().getTable("Tunables");
        
        DoubleEntry entry = table.getDoubleTopic(id).getEntry(initVal);
        table.getDoubleTopic(id).setPersistent(true);
        entry.accept(initVal);
        return entry;
    }

    public static DoubleEntry[] createSendableChooser(String subTable, String[] items, double[] initVals){
        DoubleEntry[] entry = new DoubleEntry[items.length];
        NetworkTable table = NetworkTableInstance.getDefault().getTable("Tunables").getSubTable(subTable);
        
        for (int i = 0; i < Math.min(items.length, initVals.length); i++){
            entry[i] = table.getDoubleTopic(items[i]).getEntry(initVals[i]);
            table.getDoubleTopic(items[i]).setPersistent(true);
            entry[i].accept(initVals[i]);
        }
        return entry;
    }

    public static BooleanEntry createSendableChooser(String id, boolean initVal){
        NetworkTable table = NetworkTableInstance.getDefault().getTable("Tunables");
        
        BooleanEntry entry = table.getBooleanTopic(id).getEntry(initVal);
        table.getBooleanTopic(id).setPersistent(true);
        entry.accept(initVal);

        return entry;
    }

    public static BooleanEntry[] createSendableChooser(String subTable, String[] items, boolean[] initVals){
        BooleanEntry[] entry = new BooleanEntry[items.length];
        NetworkTable table = NetworkTableInstance.getDefault().getTable("Tunables").getSubTable(subTable);
        
        for (int i = 0; i < Math.min(items.length, initVals.length); i++){
            entry[i] = table.getBooleanTopic(items[i]).getEntry(initVals[i]);
            table.getBooleanTopic(items[i]).setPersistent(true);
            entry[i].accept(initVals[i]);
        }
        return entry;
    }

    /**
     * On changes it will apply the value to the consumer
     * */ 
    public static void checker(DoubleEntry entry, DoubleConsumer consumer){
        long[] lastChange = new long[]{Long.MIN_VALUE};
        new Trigger(()->{
            boolean b = entry.getLastChange() != lastChange[0];
            if (b){
                lastChange[0] = entry.getLastChange();
                consumer.accept(entry.get());
            }
            return b;
        }).whileTrue(Commands.none());
    }

    /**
     * On changes it will apply the value to the consumer
     * */ 
    public static void checker(DoubleEntry[] entry, DoubleConsumer[] consumer){
        for (int i = 0; i < Math.min(entry.length, consumer.length); i++){
            checker(entry[i], consumer[i]);
        }
    }

    /**
     * On changes it will apply the value to the consumer
     * */ 
    public static void checker(BooleanEntry entry, BooleanConsumer consumer){
        long[] lastChange = new long[]{Long.MIN_VALUE};
        new Trigger(()->{
            boolean b = entry.getLastChange() != lastChange[0];
            if (b){
                lastChange[0] = entry.getLastChange();
                consumer.accept(entry.get());
            }
            
            return b;
        }).whileTrue(Commands.none());
    }

    /**
     * On changes it will apply the value to the consumer
     * */ 
    public static void checker(BooleanEntry[] entry, BooleanConsumer[] consumer){
        for (int i = 0; i < Math.min(entry.length, consumer.length); i++){
            checker(entry[i], consumer[i]);
        }
    }
}
