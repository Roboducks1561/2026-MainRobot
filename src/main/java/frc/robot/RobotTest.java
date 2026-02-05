package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.swerve.swerveHelpers.Telemetry;

public class RobotTest extends RobotContainer{
    

  private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps;
  private double MaxAngularRate = TunerConstants.MAX_ANGULAR_RATE;

  private double speedPercent = .3;
  private double rotationPercent = .3;


  private final CommandXboxController driverController = new CommandXboxController(0);
  
  private final SwerveDrive drivetrain = new SwerveDrive();

  // private final Arm arm = new Arm();
  // private final Indexer indexer = new Indexer();
  // private final Intake intake = new Intake();
  // private final Hood hood = new Hood();
  // private final Shooter shooter = new Shooter();
  // private final Turret turret = new Turret();

  // private final ObjectDetection objectDetection = new ObjectDetection("Test",
  //   new Transform3d(new Translation3d(0,-.101, .522), new Rotation3d(0, Units.degreesToRadians(-20), Units.degreesToRadians(0))), ()->drivetrain.getPose());
  
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(0).withRotationalDeadband(0)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); 

  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

  private final Telemetry logger = new Telemetry(MaxSpeed);

  private void configureBindings() {
    drivetrain.resetPose(new Pose2d(7,5,Rotation2d.fromDegrees(180)));
  
    drivetrain.createDefaultCommand(driverController, speedPercent, rotationPercent);
    
    drivetrain.getDriveIO().registerTelemetry((log)->logger.telemeterize(log));
    
    driverController.y().onTrue(Commands.runOnce(() -> drivetrain.seedFieldRelative(drivetrain.getPose().getRotation())));

    new Trigger(()->DriverStation.isTeleop()).onTrue(Commands.runOnce(()->{
      if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red){
        drivetrain.seedFieldRelative(Rotation2d.fromDegrees(180));
      }else{
        drivetrain.seedFieldRelative(Rotation2d.fromDegrees(0));
      }
    }));
  }

  public RobotTest() {
    drivetrain.configurePathPlanner();

    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());
    DataLogManager.logNetworkTables(true);

    SmartDashboard.putData(CommandScheduler.getInstance());

    configureBindings();
  }

  public Command getAutonomousCommand() {
    return Commands.none();
  }
}
