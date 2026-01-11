package frc.robot.subsystems.defaultSystems.digitalInputs;

public class DigitalInputSim implements DigitalInputIO{

    private boolean digitalInput;

    private boolean inverted;

    public DigitalInputSim(){}

    @Override
    public boolean getValue() {
        return digitalInput ^ inverted;
    }

    @Override
    public void setValue(boolean value) {
        digitalInput = value;
    }

    @Override
    public void invert(boolean inverted) {
        this.inverted = inverted;
    }
    
}
