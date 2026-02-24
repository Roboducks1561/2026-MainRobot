package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.Rotation;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.jni.SwerveJNI;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.generated.TunerConstants;


public class CommandSwerveDrivetrain extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> implements SwerveDriveIO{
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    public CommandSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants, double odometryUpdateFrequency,
            Matrix<N3, N1> odometryStandardDeviation, Matrix<N3, N1> visionStandardDeviation,
            SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>... modules) {
        super(TalonFX::new, TalonFX::new, CANcoder::new, drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>... modules) {
        super(TalonFX::new, TalonFX::new, CANcoder::new, driveTrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }
    
    @Override
    public void seedFieldRelative(Rotation2d rot) {
        setOperatorPerspectiveForward(rot);
    }

    @Override
    public ChassisSpeeds getSpeeds() {
        return getState().Speeds;
    }

    @Override
    public Rotation2d getYaw() {
        return getState().RawHeading;
    }
    
    @Override
    public Rotation2d getYawOffset(){
        return Rotation2d.fromRadians(SwerveJNI.JNI_GetOperatorForwardDirection(m_drivetrainId));
    }

    private double resetTime = -1;
    @Override
    public void resetPose(Pose2d pose) {
        resetTime = Timer.getFPGATimestamp();
        // TODO Auto-generated method stub
        super.resetPose(pose);
    }

    @Override
    public void addVisionMeasurement(Pose2d pose, double timestep, Vector<N3> stdDev) {
        if (currentTimeToFPGA(timestep) < resetTime){
            return;
        }
        super.addVisionMeasurement(pose, timestep, stdDev);
    }

    private double currentTimeToFPGA(double currentTime) {
        return (Timer.getFPGATimestamp() - Utils.getCurrentTimeSeconds()) + currentTime;
    }

    @Override
    public Pose2d getPose() {
        return getState().Pose;
    }

    @Override
    public double getDriveMotorRotations(int num) {
        return getModule(0).getDriveMotor().getPosition().getValueAsDouble()/TunerConstants.kDriveGearRatio;
    }

    @Override
    public double getTotalAddedRotation() {
        return getPigeon2().getAccumGyroZ().asSupplier().get().in(Rotation);
    }
}
