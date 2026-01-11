package frc.robot.subsystems.defaultSystems.digitalInputs;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class MotorDI implements DigitalInputIO{

    private final BooleanSupplier valueSupplier;

    private boolean inverted;

    public MotorDI(DoubleSupplier statorCurrent, double min){
        valueSupplier = ()->statorCurrent.getAsDouble() > min;
    }

    @Override
    public boolean getValue() {
        return valueSupplier.getAsBoolean() ^ inverted;
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
    
}
