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

public class IntakeConstants {
    public static final int INTAKE_MOTOR_ID = 12;
    // Add 0.25 V output to overcome static friction

    public static final double kS = .6; // An error of 1 rps results in 0.11 V output
    public static final double kV = 0; // A velocity target of 1 rps results in 0.12 V output
    public static final double kA = 0; // An acceleration of 1 rps/s requires 0.01 V output
    public static final double kP = .1; // This will need to be tuned after feedforward
    public static final double kI = 0; // For flywheels, this should be 0
    public static final double kD = 0; // For flywheels, this should be 0

    public static final double INTAKE_SENSOR_TO_MECHANISM_RATIO = 3.49056;

    /* Intake Current Limiting */
    public static final int INTAKE_CURRENT_LIMIT = 30;
    public static final int INTAKE_SUPPLY_CURRENT_THRESHOLD = 65;
    public static final int INTAKE_CURRENT_THRESHOLD = 65;
    public static final double INTAKE_CURRENT_THRESHOLD_TIME = 0.1;
    public static final boolean INTAKE_ENABLE_CURRENT_LIMIT = true;

    public static final boolean INTAKE_STATOR_CURRENT_LIMIT_ENABLE = true;
	public static final double INTAKE_STATOR_CURRENT_LIMIT = 60;


    public static final TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();
    public static final CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs();
    static{
        Slot0Configs slot0Configs = talonFXConfiguration.Slot0;
        slot0Configs.kS = IntakeConstants.kS;
        slot0Configs.kV = IntakeConstants.kV;
        slot0Configs.kA = IntakeConstants.kA;
        slot0Configs.kP = IntakeConstants.kP;
        slot0Configs.kI = IntakeConstants.kI;
        slot0Configs.kD = IntakeConstants.kD;

        talonFXConfiguration.Feedback.SensorToMechanismRatio = IntakeConstants.INTAKE_SENSOR_TO_MECHANISM_RATIO;

        talonFXConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // set Motion Magic Velocity settings
        talonFXConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        MotionMagicConfigs motionMagicConfigs = talonFXConfiguration.MotionMagic;
        motionMagicConfigs.MotionMagicAcceleration = 9999; // Target acceleration of 400 rps/s (0.25 seconds to max)
        motionMagicConfigs.MotionMagicJerk = 0; // Target jerk of 4000 rps/s/s (0.1 seconds)
    

        currentLimits.SupplyCurrentLimit = IntakeConstants.INTAKE_CURRENT_LIMIT; // Limit to 1 amps
        currentLimits.SupplyCurrentLimitEnable = IntakeConstants.INTAKE_ENABLE_CURRENT_LIMIT; // And enable it

        currentLimits.StatorCurrentLimit = IntakeConstants.INTAKE_STATOR_CURRENT_LIMIT; // Limit stator to 20 amps
        currentLimits.StatorCurrentLimitEnable = IntakeConstants.INTAKE_STATOR_CURRENT_LIMIT_ENABLE; // And enable it

        talonFXConfiguration.CurrentLimits = currentLimits;
    }

    public static final DCMotor gearbox = DCMotor.getFalcon500(1);

    public static final FlywheelSim intakeSim = new FlywheelSim(LinearSystemId.createFlywheelSystem(gearbox,0.02,1),
     gearbox);
}
