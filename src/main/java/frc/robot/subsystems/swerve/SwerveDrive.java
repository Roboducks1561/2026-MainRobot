package frc.robot.subsystems.swerve;

import java.util.Optional;
import java.util.function.BiConsumer;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.dyn4j.geometry.Rotation;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ApplyFieldSpeeds;
import com.ctre.phoenix6.swerve.SwerveRequest.ApplyRobotSpeeds;
import com.ctre.phoenix6.swerve.SwerveRequest.SwerveDriveBrake;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.DriveFeedforwards;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.commands.PathOnTheFly;
import frc.robot.commands.PathOnTheFly.PathConfig;
import frc.robot.constants.LimelightConstants;
import frc.robot.constants.PathplannerConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.defaultSystems.vision.Vision;
import frc.robot.subsystems.defaultSystems.vision.objectDetection.ObjectDetection;
import frc.robot.subsystems.swerve.accelerometer.AccelerometerIO;
import frc.robot.subsystems.swerve.accelerometer.RoboRioAccelerometer;
import frc.robot.subsystems.swerve.accelerometer.SimAccelerometer;
import frc.robot.subsystems.swerve.simSwerve.SimSwerve;
import frc.robot.subsystems.swerve.swerveHelpers.PoseNavigation;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.MutSlewRateLimiter;

public class SwerveDrive extends SubsystemBase{
    private final Vision cameras;
    private final ObjectDetection objectDetection;
    private final AccelerometerIO accelerometer;

    private final SwerveDriveIO swerveIO;

    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();

    private final NetworkTable robot = inst.getTable("Robot");
    private final NetworkTable odom = robot.getSubTable("Odometry");

    private final StructPublisher<Pose2d> posePublisher = odom
        .getStructTopic("RobotPose", Pose2d.struct).publish();

    public final ApplyRobotSpeeds robotCentricDrive = new SwerveRequest.ApplyRobotSpeeds();
    public final ApplyFieldSpeeds fieldCentricDrive = new SwerveRequest.ApplyFieldSpeeds();
    public final SwerveDriveBrake swerveBrake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.FieldCentric driverCentricDrive = new SwerveRequest.FieldCentric()
      .withDeadband(0).withRotationalDeadband(0)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); 

    public final PIDController speedsPID = new PIDController(6, 0, 0);
    public final PIDController rotationPID = new PIDController(6, 0, 0);

    public final MutSlewRateLimiter xSlewRateLimiter = new MutSlewRateLimiter(100);
    public final MutSlewRateLimiter ySlewRateLimiter = new MutSlewRateLimiter(100);
    public final MutSlewRateLimiter rotSlewRateLimiter = new MutSlewRateLimiter(100);
    
    
    public final PoseNavigation poseNavigation;


    private final Notifier visionUpdates;


    private BiConsumer<ChassisSpeeds, DriveFeedforwards> autoConsumer;

    public SwerveDrive(){
        if (Robot.isSimulation()){
            this.swerveIO = new SimSwerve();
            this.accelerometer = new SimAccelerometer(()->getSpeeds().vxMetersPerSecond,()->getSpeeds().vyMetersPerSecond);
        }else{
            this.swerveIO = TunerConstants.createDrivetrain();
            this.accelerometer = new RoboRioAccelerometer();
        }

        autoConsumer = (speeds, feedForward)->{
            swerveIO.setControl(robotCentricDrive.withSpeeds(speeds.times(1)));
        };

        poseNavigation = new PoseNavigation(speedsPID, rotationPID);

        objectDetection = new ObjectDetection(LimelightConstants.BACKWARD_LIMELIGHT_NAME
            , LimelightConstants.BACKWARD_LIMELIGHT_CAMERA_TRANSFORM
            , ()->getPose());
        
        LimelightHelpers.setPipelineIndex(LimelightConstants.BACKWARD_LIMELIGHT_NAME, 0);
        new Trigger(()->DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red).onTrue(Commands.runOnce(()->{
            LimelightHelpers.setPipelineIndex(LimelightConstants.BACKWARD_LIMELIGHT_NAME, 0);
        }));
        new Trigger(()->DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue).onTrue(Commands.runOnce(()->{
            LimelightHelpers.setPipelineIndex(LimelightConstants.BACKWARD_LIMELIGHT_NAME, 1);
        }));

        cameras = new Vision(swerveIO, new Transform3d[]{
                LimelightConstants.FR_LIMELIGHT_CAMERA_TRANSFORM,
                LimelightConstants.FL_LIMELIGHT_CAMERA_TRANSFORM
            },
                LimelightConstants.FR_LIMELIGHT_NAME,
                LimelightConstants.FL_LIMELIGHT_NAME
        );

        visionUpdates = new Notifier(()->{
            double time = Timer.getFPGATimestamp();
            try {
                cameras.updateVisionPose();
            } catch (Exception e) {
    
            }
            if (Timer.getFPGATimestamp() - time > .03){
                System.out.println("Vision Overrun");
            }
        });
        Runtime.getRuntime().addShutdownHook(new Thread(visionUpdates::close));
        visionUpdates.startPeriodic(.02);
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return this.run(() -> setControl(requestSupplier.get()));
    }

    public void setControl(SwerveRequest request) {
        swerveIO.setControl(request);
    }

    @Override
    public void periodic(){
        posePublisher.accept(getPose());
    }

    public void resetPose(Pose2d pose){
        swerveIO.resetPose(pose);
    }

    public void seedFieldRelative(Rotation2d rot){
        swerveIO.seedFieldRelative(rot);
    }

    public void createDefaultCommand(CommandXboxController driverController, double speedPercent, double rotationPercent){
        setDefaultCommand(applyRequest(() -> driverCentricDrive.withVelocityX(-driverController.getLeftY() * speedPercent * TunerConstants.kSpeedAt12VoltsMps)
            .withVelocityY(-driverController.getLeftX() * speedPercent * TunerConstants.kSpeedAt12VoltsMps)
            .withRotationalRate(-driverController.getRightX() * rotationPercent * TunerConstants.MAX_ANGULAR_RATE)));
    }

    public BiConsumer<ChassisSpeeds, DriveFeedforwards> getAutoConsumer(){
        return autoConsumer;
    }

    public void setAutoConsumer(BiConsumer<ChassisSpeeds, DriveFeedforwards> autoConsumer){
        this.autoConsumer = autoConsumer;
    }

    private void accepter(ChassisSpeeds speeds, DriveFeedforwards feedforwards){
        autoConsumer.accept(speeds, feedforwards);
    }

    public void configurePathPlanner() {
        AutoBuilder.configure(
            this::getPose, // getState of the robot pose
            this::resetPose,  // Consumer for seeding pose against auto
            swerveIO::getSpeeds,
            (speeds, feedforwards)->accepter(speeds, feedforwards), // Consumer of ChassisSpeeds to drive the robot
            new PPHolonomicDriveController(new PIDConstants(5, 0, 0),
                                            new PIDConstants(5, 0, 0)),
            PathplannerConstants.pathingConfig,
            () -> {
                // Boolean supplier that controls when the path will be mirrored for the red alliance
                // This will flip the path being followed to the red side of the field.
                // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            },
            // Reference to this subsystem to set requirements // Change this if the path needs to be flipped on red vs blue
            this); // Subsystem for requirements
            
    }

    public ChassisSpeeds getSpeeds(){
        return swerveIO.getState().Speeds;
    }

    /**
     * As of now does not return a rotational acceleration
     * @return
     */
    public ChassisSpeeds getAcceleration(){
        return new ChassisSpeeds(accelerometer.getX(), accelerometer.getY(), 0);
    }

    public Pose2d getPose(){
        return swerveIO.getPose();
    }

    public Rotation2d getYaw(){
        return swerveIO.getYaw();
    }

    public SwerveDriveIO getDriveIO(){
        return swerveIO;
    }

    public Optional<Pose2d> getObjectPose(){
        return objectDetection.getPiecePose();
    }

    public void registerTelemetry(Consumer<SwerveDriveState> object) {
        swerveIO.registerTelemetry(object);
    }

    public Command brake(){
        return applyRequest(()->swerveBrake);
    }

    public Command stop(){
        return applyRequest(()->fieldCentricDrive.withSpeeds(new ChassisSpeeds()));
    }

    public boolean withinRotation(Rotation2d other, double maxRadError){
        Pose2d pose = getPose();
        if (Math.abs(pose.getRotation().getRadians() - other.getRadians()) > maxRadError){
            return false;
        }
        return true;
    }

    public boolean withinCoords(Pose2d wantedPose, double maxXError, double maxYError, double maxRadError){
        Pose2d pose = getPose();
        if (Math.abs(pose.getX() - wantedPose.getX()) > maxXError){
            return false;
        }
        if (Math.abs(pose.getY() - wantedPose.getY()) > maxYError){
            return false;
        }
        if (Math.abs(pose.getRotation().getRadians() - wantedPose.getRotation().getRadians()) > maxRadError){
            return false;
        }
        return true;
    }

    public boolean withinCoords(Pose2d wantedPose, double maxDistError, double maxRadError){
        Pose2d pose = getPose();
        if (Math.hypot(pose.getX() - wantedPose.getX(), pose.getY()-wantedPose.getY()) > maxDistError){
            return false;
        }
        if (Math.abs(pose.getRotation().getRadians() - wantedPose.getRotation().getRadians()) > maxRadError){
            return false;
        }
        return true;
    }

    public Command rotateTo(Rotation2d rotation2d, double maxRads){
        return applyRequest(()->fieldCentricDrive.withSpeeds(poseNavigation.calculateTowardRotation(rotation2d, getPose(), maxRads)));
    }

    public Command rotateTo(Supplier<Rotation2d> rotation2d, double maxRads){
        return applyRequest(()->fieldCentricDrive.withSpeeds(poseNavigation.calculateTowardRotation(rotation2d.get(), getPose(), maxRads)));
    }

    public Command pathToPose(Pose2d pose, double endDist, double maxSpeed){
        return PathOnTheFly.AutoToPoint.getToPoint(pose, new PathConfig(maxSpeed,5,
            Rotation2d.fromRadians(3*Math.PI),Rotation2d.fromDegrees(720),0,0))
            .until(()->getPose().minus(pose).getTranslation().getNorm() < endDist);
    }

    public Command towardPose(Pose2d pose, double maxSpeed, double maxRads){
        return applyRequest(()->fieldCentricDrive.withSpeeds(poseNavigation.calculateTowardPose(pose, getPose(),maxSpeed, maxRads)));
    }

    public Command towardPose(Supplier<Pose2d> pose, double maxSpeed, double maxRads){
        return applyRequest(()->fieldCentricDrive.withSpeeds(poseNavigation.calculateTowardPose(pose.get(), getPose(),maxSpeed, maxRads)));
    }
    
    public Command towardPoseWhilePoint(Pose2d pose, Pose2d pointTo, double maxSpeed, double maxRads){
        return applyRequest(()->fieldCentricDrive.withSpeeds(poseNavigation.calculateTowardAndPoint(pose, pointTo, getPose(), maxSpeed, maxRads)));
    }

    public Command toArcWhilePoint(Pose2d pose, Pose2d pointTo, double distFromCenter, double maxSpeed, double maxRads){
        return applyRequest(()->fieldCentricDrive.withSpeeds(poseNavigation.calculateTowardAndPointArc(pose, pointTo, getPose(), distFromCenter, maxSpeed, maxRads)));
    }

    public Command pointWhileDrive(Supplier<Rotation2d> pointTo, DoubleSupplier vx, DoubleSupplier vy, double speedPercent, double MaxSpeed, double MaxAngularRate, double rotationPercent){
        return applyRequest(()->driverCentricDrive.withVelocityX(vx.getAsDouble() * speedPercent * MaxSpeed)
            .withVelocityY(vy.getAsDouble() * speedPercent * MaxSpeed)
            .withRotationalRate(poseNavigation.calculateTowardRotation(pointTo.get(), getPose(), MaxAngularRate).omegaRadiansPerSecond * rotationPercent));
    }

    public Command pointWhileDrive(Pose2d pointTo, DoubleSupplier vx, DoubleSupplier vy, double speedPercent, double MaxSpeed, double MaxAngularRate, double rotationPercent){
        return applyRequest(()->driverCentricDrive.withVelocityX(vx.getAsDouble() * speedPercent * MaxSpeed)
            .withVelocityY(vy.getAsDouble() * speedPercent * MaxSpeed)
            .withRotationalRate(poseNavigation.calculateTowardRotation(pointTo, getPose(), MaxAngularRate).omegaRadiansPerSecond * rotationPercent));
    }

    public Command toPose(Pose2d pose, double straightDist, double maxSpeed, double maxRads){
        return pathToPose(pose, straightDist, maxSpeed)
            .andThen(towardPose(pose, maxSpeed, maxRads));
    }

    public Command toPose(Pose2d pose, double straightDist, double maxSpeed, double maxRads, double straightMaxSpeed, double straightMaxRads){
        return pathToPose(pose, straightDist, maxSpeed)
            .andThen(towardPose(pose, straightMaxRads, straightMaxRads));
    }
    
    
    
    public Command toPoseAndPoint(Pose2d pose, Pose2d pointTo, double straightDist, double maxSpeed, double maxRads){
        return pathToPose(pose, straightDist, maxSpeed)
            .andThen(towardPoseWhilePoint(pose, pointTo, maxSpeed, maxRads));
    }
}
