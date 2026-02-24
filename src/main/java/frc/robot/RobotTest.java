// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.constants.IndexerConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandMechanism;
import frc.robot.subsystems.TurretMechanism.Hood;
import frc.robot.subsystems.TurretMechanism.Indexer;
import frc.robot.subsystems.TurretMechanism.Shooter;
import frc.robot.subsystems.intakeMechanism.Arm;
import frc.robot.subsystems.intakeMechanism.Intake;
import frc.robot.subsystems.intakeMechanism.Spindexer;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.swerve.swerveHelpers.Telemetry;
import frc.robot.util.sysid.SysIDGenerator;

public class RobotTest extends RobotContainer{
    

  private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps;
  private double MaxAngularRate = TunerConstants.MAX_ANGULAR_RATE;

  private double speedPercent = .3;
  private double rotationPercent = .3;


  private final CommandXboxController driverController = new CommandXboxController(0);
  
  private final SwerveDrive drivetrain = new SwerveDrive();

  private final Arm arm = new Arm();
  private final Indexer leftIndexer = new Indexer(IndexerConstants.INDEXER_MOTOR_LEFT_ID, IndexerConstants.INDEXER_CAN_RANGE_LEFT_ID, false);
  private final Indexer rightIndexer = new Indexer(IndexerConstants.INDEXER_MOTOR_RIGHT_ID, IndexerConstants.INDEXER_CAN_RANGE_RIGHT_ID, true);
  private final Spindexer spindexer = new Spindexer();
  private final Intake intake = new Intake();
  private final Hood hood = new Hood();
  private final Shooter leftShooter = new Shooter(ShooterConstants.SHOOTER_MOTOR_LEFT_ID, ShooterConstants.talonFXConfigurationLeft);
  private final Shooter rightShooter = new Shooter(ShooterConstants.SHOOTER_MOTOR_RIGHT_ID, ShooterConstants.talonFXConfigurationRight);

  private final CommandMechanism commandMechanism = new CommandMechanism(arm, intake, leftIndexer, rightIndexer, leftShooter, rightShooter, spindexer, hood, drivetrain);

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

    SysIDGenerator idGenerator = SysIDGenerator.flywheelSysID(leftShooter, leftShooter.getMotor());
    driverController.a().onTrue(idGenerator.sysIdDynamic(Direction.kForward));
    driverController.b().onTrue(idGenerator.sysIdDynamic(Direction.kReverse));
    driverController.x().onTrue(idGenerator.sysIdQuasistatic(Direction.kForward));
    driverController.y().onTrue(idGenerator.sysIdQuasistatic(Direction.kReverse));
    // driverController.a().onTrue(drivetrain.brake().withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    // driverController.b().whileTrue(Commands.defer(()->drivetrain.rotateTo(GameData.getHubPose2d(), 5), Set.of(drivetrain)));
    // driverController.rightBumper().whileTrue(leftIndexer.reachGoal(10).alongWith(rightIndexer.reachGoal(-10)).alongWith(spindexer.reachGoal(10)));
    // driverController.leftBumper().whileTrue(arm.reachGoal(.22).alongWith(intake.reachGoal(20)));
    // driverController.rightTrigger().whileTrue(leftShooter.reachGoal(90).alongWith(rightShooter.reachGoal(90)));
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
