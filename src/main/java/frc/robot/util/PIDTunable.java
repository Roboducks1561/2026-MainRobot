package frc.robot.util;

import java.util.function.Consumer;

public class PIDTunable {
    private PIDTunable(){

    }
    
    public static void createPIDSGVAChooser(String id, Consumer<double[]> consumer, double[] initVals){
        SendableConsumer.createSendableChooser(id, new String[]{"P", "I", "D", "S", "G", "V", "A"}, initVals);
    }

    public static void createPIDChooser(String id, Consumer<double[]> consumer, double[] initVals){
        SendableConsumer.createSendableChooser(id, new String[]{"P", "I", "D"}, initVals);
    }
}
