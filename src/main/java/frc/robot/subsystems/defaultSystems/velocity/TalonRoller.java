package frc.robot.subsystems.defaultSystems.velocity;

import java.util.ArrayList;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

public class TalonRoller implements VelocityIO{
    
    private TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();

    //TODO again, like positional, this is the one thing that should be changeable, but as of now isn't
    private final MotionMagicVelocityVoltage torqueCurrentFOC = new MotionMagicVelocityVoltage(0);
    private final VelocityVoltage positionVoltage = new VelocityVoltage(0);
    private final VoltageOut voltageRequest = new VoltageOut(0);

    private final boolean pro;

    private final TalonFX intakeMotor;

    private final ArrayList<TalonFX> followers = new ArrayList<>();

    private double targetVelocity = 0;

    public TalonRoller(TalonFX motor, TalonFXConfiguration configuration, boolean pro){
        intakeMotor = motor;
        this.talonFXConfigs = configuration;
        this.pro = pro;
        configMotor(intakeMotor, talonFXConfigs);
    }

    @Override
    public void setVelocity(double rps) {
        targetVelocity = rps;
        if (pro){
            intakeMotor.setControl(torqueCurrentFOC.withVelocity(rps).withSlot(0));
            return;
        }
        intakeMotor.setControl(positionVoltage.withVelocity(rps).withSlot(0));
    }

    @Override
    public void setVoltage(double volts) {
        intakeMotor.setControl(voltageRequest.withOutput(volts));
    }

    @Override
    public void stop() {
        intakeMotor.stopMotor();
    }

    @Override
    public double getVelocity() {
        return intakeMotor.getVelocity().getValueAsDouble();
    }

    @Override
    public double getTarget() {
        return targetVelocity;
    }

    @Override
    public double getAcceleration() {
        return intakeMotor.getAcceleration().getValueAsDouble();
    }

    @Override
    public double getCurrent() {
        return intakeMotor.getStatorCurrent().getValueAsDouble();
    }
    
    public void configMotor(TalonFX motor, TalonFXConfiguration talonFXConfigs){
        motor.getConfigurator().apply(new TalonFXConfiguration());
        motor.getConfigurator().apply(talonFXConfigs);
    }

    public TalonRoller withFollower(TalonFX motor, boolean opposeDirection){
        MotorAlignmentValue alignmentValue = opposeDirection ? MotorAlignmentValue.Opposed : MotorAlignmentValue.Aligned;
        motor.setControl(new Follower(this.intakeMotor.getDeviceID(), alignmentValue));
        followers.add(motor);
        configMotor(motor, talonFXConfigs);
        return this;
    }
}
