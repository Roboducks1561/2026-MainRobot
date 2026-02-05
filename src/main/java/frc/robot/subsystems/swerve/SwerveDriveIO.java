package frc.robot.subsystems.swerve;

import java.util.function.Consumer;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N3;

public interface SwerveDriveIO {

    public void setControl(SwerveRequest request);

    public Pose2d getPose();

    public void resetPose(Pose2d pose);

    public void seedFieldRelative(Rotation2d rot);

    public void addVisionMeasurement(Pose2d pose, double timestep, Vector<N3> stdDev);
    
    public void registerTelemetry(Consumer<SwerveDriveState> state);

    public Rotation2d getYaw();
    
    public Rotation2d getYawOffset();

    public ChassisSpeeds getSpeeds();

    public SwerveDriveState getState();

    //This is a test thing, probably shouldn't stay
    public double getDriveMotorRotations(int num);

    public double getTotalAddedRotation();
}
