package frc.robot.constants;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class IndexerConstants {
    public static final int INDEXER_MOTOR_LEFT_ID = 22;
    public static final int INDEXER_MOTOR_RIGHT_ID = 24;
    // Add 0.25 V output to overcome static friction

    public static final double kS = 0.3; // An error of 1 rps results in 0.11 V output
    public static final double kV = 0.355; // A velocity target of 1 rps results in 0.12 V output
    public static final double kA = 0; // An acceleration of 1 rps/s requires 0.01 V output
    public static final double kP = 0; // This will need to be tuned after feedforward
    public static final double kI = 0; // For flywheels, this should be 0
    public static final double kD = 0; // For flywheels, this should be 0

    public static final double INDEXER_SENSOR_TO_MECHANISM_RATIO = 3;
    /* _INDEXER Current Limiting */
    public static final int INDEXER_SUPPLY_CURRENT_LIMIT = 40;
    public static final double INDEXER_CURRENT_THRESHOLD_TIME = 0.1;
    public static final boolean INDEXER_ENABLE_CURRENT_LIMIT = true;

    public static final boolean INDEXER_STATOR_CURRENT_LIMIT_ENABLE = true;
	public static final double INDEXER_STATOR_CURRENT_LIMIT = 60;

    public static final int INDEXER_CAN_RANGE_LEFT_ID = 21;
    public static final int INDEXER_CAN_RANGE_RIGHT_ID = 22;

    public static final TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();
    public static final CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs();
    static{
        Slot0Configs slot0Configs = talonFXConfiguration.Slot0;
        slot0Configs.kS = kS;
        slot0Configs.kV = kV;
        slot0Configs.kA = kA;
        slot0Configs.kP = kP;
        slot0Configs.kI = kI;
        slot0Configs.kD = kD;

        talonFXConfiguration.Feedback.SensorToMechanismRatio = INDEXER_SENSOR_TO_MECHANISM_RATIO;

        talonFXConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // set Motion Magic Velocity settings
        MotionMagicConfigs motionMagicConfigs = talonFXConfiguration.MotionMagic;
        motionMagicConfigs.MotionMagicAcceleration = 9999; // Target acceleration of 400 rps/s (0.25 seconds to max)
        motionMagicConfigs.MotionMagicJerk = 0; // Target jerk of 4000 rps/s/s (0.1 seconds)
    

        currentLimits.SupplyCurrentLimit = INDEXER_SUPPLY_CURRENT_LIMIT; // Limit to 1 amps
        currentLimits.SupplyCurrentLimitEnable = INDEXER_ENABLE_CURRENT_LIMIT; // And enable it

        currentLimits.StatorCurrentLimit = INDEXER_STATOR_CURRENT_LIMIT; // Limit stator to 20 amps
        currentLimits.StatorCurrentLimitEnable = INDEXER_STATOR_CURRENT_LIMIT_ENABLE; // And enable it

        talonFXConfiguration.CurrentLimits = currentLimits;
    }

    public static final DCMotor gearbox = DCMotor.getFalcon500(1);

    public static final FlywheelSim RollerSim = new FlywheelSim(LinearSystemId.createFlywheelSystem(gearbox,0.005,.5),
     gearbox);
}
