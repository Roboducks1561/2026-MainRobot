package frc.robot.constants;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class ShooterConstants {
    public static final int SHOOTER_MOTOR_LEFT_ID = 21;
    public static final int SHOOTER_MOTOR_RIGHT_ID = 23;
    // Add 0.25 V output to overcome static friction

    //Left
    public static final double lS = .265; // An error of 1 rps results in 0.11 V output
    public static final double lV = .12; // A velocity target of 1 rps results in 0.12 V output
    public static final double lA = 0; // An acceleration of 1 rps/s requires 0.01 V output
    public static final double lP = 0; // This will need to be tuned after feedforward
    public static final double lI = 0; // For flywheels, this should be 0
    public static final double lD = 0; // For flywheels, this should be 0

    //Right
    public static final double rS = .34; // An error of 1 rps results in 0.11 V output
    public static final double rV = .123; // A velocity target of 1 rps results in 0.12 V output
    public static final double rA = 0; // An acceleration of 1 rps/s requires 0.01 V output
    public static final double rP = 0; // This will need to be tuned after feedforward
    public static final double rI = 0; // For flywheels, this should be 0
    public static final double rD = 0; // For flywheels, this should be 0

    public static final double SHOOTER_SENSOR_TO_MECHANISM_RATIO = 1;

    /* SHOOTER Current Limiting */
    public static final int SHOOTER_CURRENT_LIMIT = 90;
    public static final int SHOOTER_SUPPLY_CURRENT_THRESHOLD = 65;
    public static final int SHOOTER_CURRENT_THRESHOLD = 65;
    public static final double SHOOTER_CURRENT_THRESHOLD_TIME = 0.1;
    public static final boolean SHOOTER_ENABLE_CURRENT_LIMIT = true;

    public static final boolean SHOOTER_STATOR_CURRENT_LIMIT_ENABLE = true;
	public static final double SHOOTER_STATOR_CURRENT_LIMIT = 60;


    public static final TalonFXConfiguration talonFXConfigurationLeft = new TalonFXConfiguration();
    public static final TalonFXConfiguration talonFXConfigurationRight = new TalonFXConfiguration();
    public static final CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs();
    static{
        Slot0Configs slot0ConfigsL = talonFXConfigurationLeft.Slot0;
        slot0ConfigsL.kS = lS;
        slot0ConfigsL.kV = lV;
        slot0ConfigsL.kA = lA;
        slot0ConfigsL.kP = lP;
        slot0ConfigsL.kI = lI;
        slot0ConfigsL.kD = lD;

        talonFXConfigurationLeft.Feedback.SensorToMechanismRatio = SHOOTER_SENSOR_TO_MECHANISM_RATIO;

        talonFXConfigurationLeft.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        talonFXConfigurationLeft.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        // set Motion Magic Velocity settings
        MotionMagicConfigs motionMagicConfigs = talonFXConfigurationLeft.MotionMagic;
        motionMagicConfigs.MotionMagicAcceleration = 9999; // Target acceleration of 400 rps/s (0.25 seconds to max)
        motionMagicConfigs.MotionMagicJerk = 0; // Target jerk of 4000 rps/s/s (0.1 seconds)
    

        currentLimits.SupplyCurrentLimit = SHOOTER_CURRENT_LIMIT; // Limit to 1 amps
        currentLimits.SupplyCurrentLimitEnable = SHOOTER_ENABLE_CURRENT_LIMIT; // And enable it

        currentLimits.StatorCurrentLimit = SHOOTER_STATOR_CURRENT_LIMIT; // Limit stator to 20 amps
        currentLimits.StatorCurrentLimitEnable = SHOOTER_STATOR_CURRENT_LIMIT_ENABLE; // And enable it

        talonFXConfigurationLeft.CurrentLimits = currentLimits;


        Slot0Configs slot0ConfigsR = talonFXConfigurationRight.Slot0;
        slot0ConfigsR.kS = rS;
        slot0ConfigsR.kV = rV;
        slot0ConfigsR.kA = rA;
        slot0ConfigsR.kP = rP;
        slot0ConfigsR.kI = rI;
        slot0ConfigsR.kD = rD;

        talonFXConfigurationRight.Feedback = talonFXConfigurationLeft.Feedback;
        talonFXConfigurationRight.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        talonFXConfigurationRight.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        talonFXConfigurationRight.MotionMagic = talonFXConfigurationLeft.MotionMagic;
        talonFXConfigurationRight.CurrentLimits = talonFXConfigurationLeft.CurrentLimits;
    }

    public static final DCMotor gearbox = DCMotor.getFalcon500(1);

    public static final FlywheelSim shooterSim = new FlywheelSim(LinearSystemId.createFlywheelSystem(gearbox,1,10),
     gearbox);
}
