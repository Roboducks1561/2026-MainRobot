package frc.robot.constants;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class TurretConstants{

    public static final double TURRET_WEIGHT_KG = 1;
    public static final double TURRET_LENGTH_METERS = .2;
    public static final double MAX_TURRET_ANGLE_RAD = Units.rotationsToRadians(.5676);
    public static final double MIN_TURRET_ANGLE_RAD = Units.rotationsToRadians(-.5676);

    public static final int TURRET_MOTOR_ID = 20;
    public static final int TURRET_ENCODER_ID = 20;

    public static final double kS = 0;
    public static final double kV = 0;
    public static final double kA = 0;
    
    public static final double kP = 40;
    public static final double kI = 0;
    public static final double kD = .1;//50;
    public static final double kG = 0;//-26;//8.4749;

    public static final double TURRET_SENSOR_TO_MECHANISM_RATIO = 9.58333*5;
    public static final double TURRET_ROTOR_TO_SENSOR_RATIO = 1;

    public static final double CRUISE_VELOCITY = 9999;
    public static final double MAX_ACCELERATION = 9999;
    public static final double JERK = 0;

    
    public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromRotations(-.4638);

    /* TURRET Current Limiting */ //TODO: Change/Fix these values
    public static final int TURRET_SUPPLY_CURRENT_LIMIT = 20;
    public static final boolean TURRET_ENABLE_CURRENT_LIMIT = true;
    public static final boolean TURRET_STATOR_CURRENT_LIMIT_ENABLE = true;
    public static final double TURRET_STATOR_CURRENT_LIMIT = 40;



    public static final TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();
    public static final CANcoderConfiguration encoderConfiguration = new CANcoderConfiguration();
    public static final CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs();
    static{
        encoderConfiguration.MagnetSensor.AbsoluteSensorDiscontinuityPoint = .5;
        // canCoderConfigs.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        encoderConfiguration.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        encoderConfiguration.MagnetSensor.MagnetOffset = ANGLE_OFFSET.getRotations();

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
        talonFXConfiguration.Feedback.FeedbackRemoteSensorID = TURRET_ENCODER_ID;
        talonFXConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        talonFXConfiguration.Feedback.SensorToMechanismRatio = TURRET_SENSOR_TO_MECHANISM_RATIO;
        talonFXConfiguration.Feedback.RotorToSensorRatio = TURRET_ROTOR_TO_SENSOR_RATIO;
        
        /* Motion Magic Settings */
        var motionMagicConfigs = talonFXConfiguration.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = CRUISE_VELOCITY; // Unlimited cruise velocity
        motionMagicConfigs.MotionMagicAcceleration = MAX_ACCELERATION;
        motionMagicConfigs.MotionMagicJerk = JERK;
        // motionMagicConfigs.MotionMagicExpo_kV = 0.12; // kV is around 0.12 V/rps
        // motionMagicConfigs.MotionMagicExpo_kA = 0.1; // Use a slower kA of 0.1 V/(rps/s
        /* Current Limits */
        currentLimits.SupplyCurrentLimit = TURRET_SUPPLY_CURRENT_LIMIT; // Limit to 1 amps
        // currentLimits.SupplyCurrentThreshold = TURRET_SUPPLY_CURRENT_THRESHOLD; // If we exceed 4 amps
        // currentLimits.SupplyTimeThreshold = TURRET_CURRENT_THRESHOLD_TIME; // For at least 1 second
        currentLimits.SupplyCurrentLimitEnable = TURRET_ENABLE_CURRENT_LIMIT; // And enable it
        currentLimits.StatorCurrentLimit = TURRET_STATOR_CURRENT_LIMIT; // Limit stator to 20 amps
        currentLimits.StatorCurrentLimitEnable = TURRET_STATOR_CURRENT_LIMIT_ENABLE; // And enable it

        talonFXConfiguration.CurrentLimits = currentLimits;

        talonFXConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    }

    public static final DCMotor gearbox = DCMotor.getFalcon500(1);

    public static final SingleJointedArmSim singleJointedArmSim =
      new SingleJointedArmSim(
          gearbox,
          TURRET_ROTOR_TO_SENSOR_RATIO * TURRET_SENSOR_TO_MECHANISM_RATIO,
          SingleJointedArmSim.estimateMOI(TURRET_LENGTH_METERS, TURRET_WEIGHT_KG),
          TURRET_LENGTH_METERS,
          MIN_TURRET_ANGLE_RAD,
          MAX_TURRET_ANGLE_RAD,
          false,
          Units.rotationsToRadians(0)
        );
}