package frc.robot.util;

import java.util.function.Consumer;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//TODO, as of now only the individual creator works, fix that!!!
public class SendableConsumer {
    private SendableConsumer(){

    }
    public static void createSendableChooser(String id, Consumer<Double> consumer, double initVal){
        Notifier notifier;

        // NetworkTableEntry entry;
        // NetworkTable table = NetworkTableInstance.getDefault().getTable("SmartDashboard");
        SmartDashboard.putNumber(id, initVal);

        notifier = new Notifier(()->{
            consumer.accept(SmartDashboard.getNumber(id, initVal));
        });
        notifier.startPeriodic(0.05);

        Runtime.getRuntime().addShutdownHook(new Thread(notifier::close));
    }

    public static void createSendableChooser(String id, String[] items, Consumer<double[]> consumers, double[] initVals){

        Notifier notifier;

        NetworkTableEntry[] entry = new NetworkTableEntry[items.length];
        NetworkTable table = NetworkTableInstance.getDefault().getTable("SmartDashboard").getSubTable(id);

        for (int i = 0; i < items.length; i++){
            entry[i] = table.getEntry(items[i]);
            entry[i].setDefaultDouble(initVals[i]);
        }

        notifier = new Notifier(()->{
            double[] values = new double[entry.length];
            for (int i = 0; i < entry.length; i++){
                values[i] = entry[i].getDouble(initVals[i]);
            }
            consumers.accept(values);
        });
        notifier.startPeriodic(0.05);

        Runtime.getRuntime().addShutdownHook(new Thread(notifier::close));
    }

    public static void createSendableChooser(String id, String[] items, Consumer<Double>[] consumer, double[] initVals){
        Notifier notifier;

        NetworkTableEntry[] entry = new NetworkTableEntry[items.length];
        NetworkTable table = NetworkTableInstance.getDefault().getTable("SmartDashboard").getSubTable(id);
        
        for (int i = 0; i < items.length; i++){
            entry[i] = table.getEntry(items[i]);
        }

        notifier = new Notifier(()->{
            for (int i = 0; i < entry.length; i++){
                consumer[i].accept(entry[i].getDouble(initVals[i]));
            }
        });
        notifier.startPeriodic(0.05);

        Runtime.getRuntime().addShutdownHook(new Thread(notifier::close));
    }

}
