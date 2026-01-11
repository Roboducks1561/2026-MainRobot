package frc.robot.subsystems.defaultSystems.digitalInputs;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;

public class CANRange implements DigitalInputIO{

    private final CANrange digitalInput;

    private boolean inverted;
    private double minDist;

    public CANRange(int id, double dist, String line){
        minDist = dist;
        digitalInput = new CANrange(id,line);
        configureCANRange();
    }

    @Override
    public boolean getValue() {
        if (digitalInput.getDistance().getValueAsDouble() == 0){
            return false;
        }
        return (digitalInput.getDistance().getValueAsDouble() <= minDist) ^ inverted;
    }

    /**
     * unused
     */
    @Override
    public void setValue(boolean value) {
        
    }

    @Override
    public void invert(boolean inverted) {
        this.inverted = inverted;
    }

    public void configureCANRange(){
        CANrangeConfiguration configuration = new CANrangeConfiguration();
        digitalInput.getConfigurator().apply(configuration);
    }
}
