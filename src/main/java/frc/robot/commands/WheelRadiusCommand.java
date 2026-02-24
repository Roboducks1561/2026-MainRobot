package frc.robot.commands;

import static edu.wpi.first.units.Units.Inches;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveRequest.ApplyRobotSpeeds;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.swerve.SwerveDrive;

public class WheelRadiusCommand extends Command {

    private final SwerveDrive drivetrain;

    private final double[] initWheelRot = new double[4];

    private boolean yawInitialized = false;
    private double lastYawWrapped = 0.0;
    private double totalYawRot = 0.0;

    private double startTime;
    private boolean finished = false;

    private static final double SPIN_RATE_RAD_PER_SEC =
            Units.degreesToRadians(60);
    private static final double RUN_TIME_SEC = 15.0;

    private final ApplyRobotSpeeds spinRequest =
            new ApplyRobotSpeeds();

    public WheelRadiusCommand(SwerveDrive drivetrain) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    private double getUnwrappedYawRotations() {
        double yaw = drivetrain
                .getDriveIO()
                .getYaw()
                .getRotations();

        if (!yawInitialized) {
            lastYawWrapped = yaw;
            yawInitialized = true;
            return totalYawRot;
        }

        double delta = yaw - lastYawWrapped;

        if (delta > 0.5)  delta -= 1.0;
        if (delta < -0.5) delta += 1.0;

        totalYawRot += delta;
        lastYawWrapped = yaw;

        return totalYawRot;
    }

    private double getAverageWheelRotations() {
        double sum = 0.0;
        for (int i = 0; i < 4; i++) {
            sum += Math.abs(
                drivetrain.getDriveIO().getDriveMotorRotations(i)
                - initWheelRot[i]
            );
        }
        return sum / 4.0;
    }

    @Override
    public void initialize() {

        for (int i = 0; i < 4; i++) {
            initWheelRot[i] =
                drivetrain.getDriveIO().getDriveMotorRotations(i);
        }

        yawInitialized = false;
        totalYawRot = 0.0;

        startTime = Utils.getCurrentTimeSeconds();
        finished = false;

        drivetrain.setControl(
            spinRequest.withSpeeds(
                new ChassisSpeeds(0, 0, SPIN_RATE_RAD_PER_SEC)
            )
        );
    }

    @Override
    public void execute() {
        System.out.println(getUnwrappedYawRotations() +"   "+getAverageWheelRotations());
        drivetrain.setControl(
            spinRequest.withSpeeds(
                new ChassisSpeeds(0, 0, SPIN_RATE_RAD_PER_SEC)
            )
        );

        if (Utils.getCurrentTimeSeconds() - startTime >= RUN_TIME_SEC) {
            finishCalibration();
        }
    }

    private void finishCalibration() {

        drivetrain.setControl(
            spinRequest.withSpeeds(new ChassisSpeeds())
        );

        double robotRotations = Math.abs(getUnwrappedYawRotations());
        double wheelRotations = getAverageWheelRotations();

        if (robotRotations < 1e-3 || wheelRotations < 1e-3) {
            System.out.println("Wheel radius calibration failed");
            finished = true;
            return;
        }

        double robotRadians = robotRotations * 2.0 * Math.PI;
        double wheelRadians = wheelRotations * 2.0 * Math.PI;

        double drivebaseRadiusMeters =
            Units.inchesToMeters(
                Math.sqrt(
                    TunerConstants.kFrontLeftXPos.in(Inches)
                    * TunerConstants.kFrontLeftXPos.in(Inches)
                    +
                    TunerConstants.kFrontLeftYPos.in(Inches)
                    * TunerConstants.kFrontLeftYPos.in(Inches)
                )
            );

        double wheelRadiusMeters =
            (robotRadians * drivebaseRadiusMeters) / wheelRadians;

        System.out.println("===== Wheel Radius Calibration =====");
        System.out.println("Robot rotations: " + robotRotations);
        System.out.println("Wheel rotations: " + wheelRotations);
        System.out.println("Wheel radius (m): " + wheelRadiusMeters);
        System.out.println("Wheel radius (in): " +
                Units.metersToInches(wheelRadiusMeters));

        finished = true;
    }

    @Override
    public boolean isFinished() {
        return finished;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(spinRequest.withSpeeds(new ChassisSpeeds()));
    }
}