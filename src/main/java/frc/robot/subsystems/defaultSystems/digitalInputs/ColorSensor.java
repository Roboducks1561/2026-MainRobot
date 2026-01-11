package frc.robot.subsystems.defaultSystems.digitalInputs;

import java.util.function.BooleanSupplier;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorSensorV3.RawColor;

import edu.wpi.first.wpilibj.I2C.Port;

public class ColorSensor implements DigitalInputIO{

    /**
     * Example of I2C port
     */
    // private final I2C.Port i2cPort = I2C.Port.kOnboard;

    private final ColorSensorV3 m_colorSensor;

    private boolean inverted;

    private BooleanSupplier boolSupplier = ()->false;

    public ColorSensor(Port id){
        m_colorSensor = new ColorSensorV3(id);
    }

    public void assignBooleanSupplier(BooleanSupplier supplier){
        boolSupplier = supplier;
    }

    @Override
    public boolean getValue() {
        return boolSupplier.getAsBoolean() ^ inverted;
    }

    public RawColor getColor(){
        return m_colorSensor.getRawColor();
    }

    public int getRed(){
        return m_colorSensor.getRed();
    }

    public int getGreen(){
        return m_colorSensor.getGreen();
    }

    public int getBlue(){
        return m_colorSensor.getBlue();
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
