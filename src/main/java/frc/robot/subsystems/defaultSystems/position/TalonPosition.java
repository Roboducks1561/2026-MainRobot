package frc.robot.subsystems.defaultSystems.position;

import java.util.ArrayList;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

public class TalonPosition implements PositionIO{

    private final TalonFXConfiguration configuration;

    private final VoltageOut voltageRequest = new VoltageOut(0);
    //TODO, the only thing in here that should be customizable and isn't. I don't currently have a fix, but i might not need one
    private final MotionMagicTorqueCurrentFOC controlRequest = new MotionMagicTorqueCurrentFOC(0);
    private final PositionVoltage positionVoltage = new PositionVoltage(0);

    private final boolean pro;

    private final TalonFX armMotor;
    private CANcoder encoder = null;

    private final ArrayList<TalonFX> followers = new ArrayList<>();

    private double armSetpointReal = 0;
    private double armFakeOffset = 0;

    public TalonPosition(TalonFX armMotor, TalonFXConfiguration configuration, boolean pro){
        this.armMotor = armMotor;
        this.configuration = configuration;
        this.pro = pro;
        configMotor(armMotor, configuration);
    }
    
    @Override
    public void setVoltage(double volts) {
        armMotor.setControl(voltageRequest.withOutput(volts));
    }
    
    @Override
    public void setPosition(double position) {
        armSetpointReal = position;
        if (pro){
            armMotor.setControl(controlRequest.withPosition(armSetpointReal - armFakeOffset).withSlot(0));
            return;
        }
        armMotor.setControl(positionVoltage.withPosition(armSetpointReal - armFakeOffset).withSlot(0));
    }

    @Override
    public void stop() {
        armMotor.stopMotor();
    }

    @Override
    public double getPosition() {
        return armMotor.getPosition().getValueAsDouble()+armFakeOffset;
    }

    @Override
    public double getTarget() {
        //This armSetpointReal occurs because motionMagic, or some PID thing in general, moves the readable setpoint slowly, likely to account for acceleration and jerk
        return armSetpointReal;//armMotor.getClosedLoopReference().getValueAsDouble()+armPretendOffset;
    }

    public void configMotor(TalonFX motor, TalonFXConfiguration talonFXConfigs){
        motor.getConfigurator().apply(new TalonFXConfiguration());
        motor.getConfigurator().apply(talonFXConfigs);
    }

    public TalonPosition withEncoder(CANcoder encoder, CANcoderConfiguration encoderConfiguration){
        this.encoder = encoder;
        encoder.getConfigurator().apply(new CANcoderConfiguration());
        encoder.getConfigurator().apply(encoderConfiguration);
        return this;
    }

    public TalonPosition withFollower(TalonFX motor, boolean opposeDirection){
        motor.setControl(new Follower(this.armMotor.getDeviceID(), opposeDirection));
        followers.add(motor);
        // configMotor(motor, configuration);
        return this;
    }

    public TalonPosition withFakeOffset(double offset){
        armFakeOffset = offset;
        return this;
    }
}
