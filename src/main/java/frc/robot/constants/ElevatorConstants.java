package frc.robot.constants;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ElevatorConstants{

    public static final int ELEVATOR_LEADER_ID = 20;
    public static final int ELEVATOR_FOLLOWER_ID = 21;

    public static final double CARRIAGE_POSITION = .2;
    public static final double STAGE_ONE_POSITION = .4;
    public static final double STAGE_TWO_POSITION = .8;

    public static final double kP = 1000;
    public static final double kI = 10;
    public static final double kD = 60;

    public static final double kS = 0.0;
    public static final double kG = 29;
    public static final double kV = 0;
    public static final double kA = 0.0;
    
    public static final double ELEVATOR_DRUM_RADIUS = .06;

    public static final double DRUM_CIRCUMFRANCE_METERS = 2 * Math.PI * ELEVATOR_DRUM_RADIUS;
    public static final double TRUE_ELEVATOR_SENSOR_TO_MECHANISM_RATIO = (25/3.0);
    public static final double ELEVATOR_SENSOR_TO_MECHANISM_RATIO = TRUE_ELEVATOR_SENSOR_TO_MECHANISM_RATIO * 2 / DRUM_CIRCUMFRANCE_METERS;
    public static final double ELEVATOR_ROTOR_TO_SENSOR_RATIO = 1;

    public static final double ELEVATOR_WEIGHT_KG = 20;

    public static final double ELEVATOR_MIN_HEIGHT = 0;
    public static final double ELEVATOR_MAX_HEIGHT = 4;

    public static final double CRUISE_VELOCITY = 100;
    public static final double MAX_ACCELERATION = 100;
	  public static final double JERK = 128;

    /* Elevator Current Limiting */ //TODO: Change/Fix these values
    public static final int ELEVATOR_CURRENT_LIMIT = 40;
    public static final int ELEVATOR_SUPPLY_CURRENT_THRESHOLD = 65;
    public static final int ELEVATOR_CURRENT_THRESHOLD = 70;
    public static final double ELEVATOR_CURRENT_THRESHOLD_TIME = 0.1;
    public static final boolean ELEVATOR_ENABLE_CURRENT_LIMIT = true;
    public static final boolean ELEVATOR_STATOR_CURRENT_LIMIT_ENABLE = true;
    public static final double ELEVATOR_STATOR_CURRENT_LIMIT = 120;



    public static final TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();
    public static final CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs();
    static{        
        Slot0Configs slot0Configs = talonFXConfiguration.Slot0;
        slot0Configs.kS = kS; // Add 0.25 V output to overcome static friction
        slot0Configs.kV = kV; // A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kA = kA; // An acceleration of 1 rps/s requires 0.01 V output
        slot0Configs.kP = kP; //4080.564;// A position error of 2.5 rotations results in 12 V output
        slot0Configs.kI = kI; //0; // no output for integrated error
        slot0Configs.kD = kD; //60.082; // A velocity error of 1 rps results in 0.1 V output
        slot0Configs.kG = kG;
        slot0Configs.GravityType = GravityTypeValue.Elevator_Static;

        talonFXConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;//InvertedValue.Clockwise_Positive
        talonFXConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        talonFXConfiguration.Feedback.SensorToMechanismRatio = ELEVATOR_SENSOR_TO_MECHANISM_RATIO;
        talonFXConfiguration.Feedback.RotorToSensorRatio = ELEVATOR_ROTOR_TO_SENSOR_RATIO;
        
        /* Motion Magic Settings */
        var motionMagicConfigs = talonFXConfiguration.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = CRUISE_VELOCITY; // Unlimited cruise velocity
        motionMagicConfigs.MotionMagicAcceleration = MAX_ACCELERATION;
        motionMagicConfigs.MotionMagicJerk = JERK;
        // motionMagicConfigs.MotionMagicExpo_kV = 0.12; // kV is around 0.12 V/rps
        // motionMagicConfigs.MotionMagicExpo_kA = 0.1; // Use a slower kA of 0.1 V/(rps/s
        /* Current Limits */
        currentLimits.SupplyCurrentLimit = ELEVATOR_CURRENT_LIMIT; // Limit to 1 amps
        // currentLimits.SupplyCurrentThreshold = ELEVATOR_SUPPLY_CURRENT_THRESHOLD; // If we exceed 4 amps
        // currentLimits.SupplyTimeThreshold = ELEVATOR_CURRENT_THRESHOLD_TIME; // For at least 1 second
        currentLimits.SupplyCurrentLimitEnable = ELEVATOR_ENABLE_CURRENT_LIMIT; // And enable it
        currentLimits.StatorCurrentLimit = ELEVATOR_STATOR_CURRENT_LIMIT; // Limit stator to 20 amps
        currentLimits.StatorCurrentLimitEnable = ELEVATOR_STATOR_CURRENT_LIMIT_ENABLE; // And enable it

        talonFXConfiguration.CurrentLimits = currentLimits;

        talonFXConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    }

    public static final DCMotor gearbox = DCMotor.getFalcon500(1);

    public static final ElevatorSim elevatorSim =
      new ElevatorSim(
          gearbox,
          ElevatorConstants.TRUE_ELEVATOR_SENSOR_TO_MECHANISM_RATIO * ElevatorConstants.ELEVATOR_ROTOR_TO_SENSOR_RATIO, //The *2 is a lie, sorry
          ElevatorConstants.ELEVATOR_WEIGHT_KG,
          ElevatorConstants.ELEVATOR_DRUM_RADIUS,
          ElevatorConstants.ELEVATOR_MIN_HEIGHT,
          ElevatorConstants.ELEVATOR_MAX_HEIGHT,
          true,
          0);
}