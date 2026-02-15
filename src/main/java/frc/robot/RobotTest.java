// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import java.util.function.Function;
import java.util.stream.Stream;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.SequentialAutos;
import frc.robot.commands.WheelRadiusCommand;
import frc.robot.constants.GameData;
import frc.robot.constants.IndexerConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandMechanism;
import frc.robot.subsystems.GameState;
import frc.robot.subsystems.TurretMechanism.Hood;
import frc.robot.subsystems.TurretMechanism.Indexer;
import frc.robot.subsystems.TurretMechanism.Shooter;
import frc.robot.subsystems.TurretMechanism.Turret;
import frc.robot.subsystems.climbMechanism.ClimbElevator;
import frc.robot.subsystems.intakeMechanism.Arm;
import frc.robot.subsystems.intakeMechanism.Intake;
import frc.robot.subsystems.intakeMechanism.Spindexer;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.swerve.swerveHelpers.Telemetry;
import frc.robot.util.ChoreoEX;
import frc.robot.util.MutSlewRateLimiter;
import frc.robot.util.PoseEX;

public class RobotTest extends RobotContainer{
    

  private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps;
  private double MaxAngularRate = TunerConstants.MAX_ANGULAR_RATE;

  private double speedPercent = .3;
  private double rotationPercent = .3;


  private final CommandXboxController driverController = new CommandXboxController(0);
  
  private final SwerveDrive drivetrain = new SwerveDrive();

  private final Arm arm = new Arm();
  private final Indexer leftIndexer = new Indexer(IndexerConstants.INDEXER_MOTOR_LEFT_ID, IndexerConstants.INDEXER_CAN_RANGE_LEFT_ID);
  private final Indexer rightIndexer = new Indexer(IndexerConstants.INDEXER_MOTOR_RIGHT_ID, IndexerConstants.INDEXER_CAN_RANGE_RIGHT_ID);
  private final Spindexer spindexer = new Spindexer();
  private final Intake intake = new Intake();
  private final Hood hood = new Hood();
  private final Shooter leftShooter = new Shooter(ShooterConstants.SHOOTER_MOTOR_LEFT_ID);
  private final Shooter rightShooter = new Shooter(ShooterConstants.SHOOTER_MOTOR_RIGHT_ID);

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

    driverController.a().onTrue(drivetrain.brake().withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    driverController.b().whileTrue(drivetrain.rotateTo(GameData.getHubPose2d(), 5));
    driverController.rightBumper().whileTrue(leftIndexer.reachGoal(5).alongWith(rightIndexer.reachGoal(5)).alongWith(spindexer.reachGoal(5)));
    driverController.leftBumper().onTrue(arm.reachGoal(.25).alongWith(intake.reachGoal(10)));
    driverController.rightTrigger().whileTrue(leftShooter.reachGoal(50).alongWith(rightShooter.reachGoal(50)));
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
