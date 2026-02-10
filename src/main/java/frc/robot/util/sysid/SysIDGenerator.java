package frc.robot.util.sysid;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

//TODO develop more
public class SysIDGenerator {

    private SysIDGenerator(){

    }

    private VoltageOut m_sysIdControl = new VoltageOut(0);

    private SysIdRoutine m_sysIdRoutine;

    private BooleanSupplier breaks = ()-> false;

    /**
     * works with single motor and same input motors, else make your own or expand on this class
     * @param requirements
     * @param motors
     * @return
     */
    public static SysIDGenerator flywheelSysID(Subsystem requirements, TalonFX... motors) {
        SysIDGenerator sysIDGenerated = generateBaseSysID(motors);

        sysIDGenerated.m_sysIdRoutine =
            new SysIdRoutine(
                new SysIdRoutine.Config(
                    null,         // Use default ramp rate (1 V/s)
                    Volts.of(4), // Reduce dynamic voltage to 4 to prevent brownout
                    null,          // Use default timeout (10 s)
                                        // Log state with Phoenix SignalLogger class
                    state -> {
                        SignalLogger.writeString("state", state.toString());
                        // for(int i = 0; i < motors.length; i++){
                        //     SignalLogger.writeDouble("motorVelocity"+i, motors[i].getVelocity().getValueAsDouble());
                        //     SignalLogger.writeDouble("motorPosition"+i, motors[i].getPosition().getValueAsDouble());
                        //     SignalLogger.writeDouble("motorVoltage"+i, motors[i].getMotorVoltage().getValueAsDouble());
                        // }
                    }
                ),
                new SysIdRoutine.Mechanism(
                    volts -> {
                        for (TalonFX motor : motors){
                            motor.setControl(sysIDGenerated.m_sysIdControl.withOutput(volts));
                        }
                    },
                    log-> {
                        for (TalonFX motor : motors){
                            log.motor("MotorSysID")
                            .voltage(Volts.mutable(0).mut_replace(motor.getMotorVoltage().getValueAsDouble(), Volts))
                            .angularPosition(Radians.mutable(0).mut_replace(motor.getPosition().getValueAsDouble(), Rotations))
                            .angularVelocity(RadiansPerSecond.mutable(0).mut_replace(motor.getVelocity().getValueAsDouble(), RadiansPerSecond));
                        }
                    },
                    requirements
                )
            );
        return sysIDGenerated;
    }

    private static SysIDGenerator generateBaseSysID(TalonFX... motors){
        SysIDGenerator sysIDGenerated = new SysIDGenerator();
        TalonFXConfiguration cfg = new TalonFXConfiguration();
        // Set any necessary configs in the Feedback group here
        for (TalonFX motor : motors){
            motor.getConfigurator().apply(cfg);
        

            /* Speed up signals for better characterization data */
            BaseStatusSignal.setUpdateFrequencyForAll(250,
                motor.getPosition(),
                motor.getVelocity(),
                motor.getMotorVoltage());

            /* Optimize out the other signals, since they're not useful for SysId */
            motor.optimizeBusUtilization();
        }

        /* Start the signal logger */
        SignalLogger.start();
        return sysIDGenerated;
    }

    public void applyStops(BooleanSupplier breakPoint){
        breaks = breakPoint;
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.quasistatic(direction).until(breaks);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.dynamic(direction).until(breaks);
    }
}
