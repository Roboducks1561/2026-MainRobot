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
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.SequentialAutos;
import frc.robot.constants.GameData;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandMechanism;
import frc.robot.subsystems.GameState;
import frc.robot.subsystems.TurretMechanism.Hood;
import frc.robot.subsystems.TurretMechanism.Shooter;
import frc.robot.subsystems.TurretMechanism.Turret;
import frc.robot.subsystems.climbMechanism.ClimbElevator;
import frc.robot.subsystems.intakeMechanism.Arm;
import frc.robot.subsystems.intakeMechanism.Indexer;
import frc.robot.subsystems.intakeMechanism.Intake;
import frc.robot.subsystems.intakeMechanism.Spindexer;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.swerve.swerveHelpers.Telemetry;
import frc.robot.util.ChoreoEX;
import frc.robot.util.MutSlewRateLimiter;


public class RobotMain extends RobotContainer {

  private SendableChooser<Command> autoChooser;

  private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps;
  private double MaxAngularRate = TunerConstants.MAX_ANGULAR_RATE;

  private double speedPercent = 1;
  private double rotationPercent = 1;


  private final CommandXboxController driverController = new CommandXboxController(0);
  
  private final SwerveDrive drivetrain = new SwerveDrive();


  private final NetworkTable robot = NetworkTableInstance.getDefault().getTable("Robot");

  private final StructPublisher<Pose2d> basePublisher = robot
    .getStructTopic("Base", Pose2d.struct).publish();

  private final Arm arm = new Arm();
  private final Indexer indexer = new Indexer();
  private final Spindexer spindexer = new Spindexer();
  private final Intake intake = new Intake();
  private final Hood hood = new Hood();
  private final Shooter shooter = new Shooter();
  // private final Turret turret = new Turret();
  private final ClimbElevator climbElevator = new ClimbElevator();

  private final CommandMechanism commandMechanism = new CommandMechanism(arm,intake,indexer,spindexer,hood,shooter,climbElevator,drivetrain);
  private final GameState gameState = new GameState(commandMechanism, driverController);

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
    })
    .alongWith(commandMechanism.climbUp().withTimeout(3).andThen(commandMechanism.climbDown())));

    // driverController.a().whileTrue(commandMechanism.passDynamic(true));
    // driverController.b().whileTrue(commandMechanism.passStatic(false));
    // driverController.x().whileTrue(commandMechanism.shootDynamic());
    // driverController.leftBumper().whileTrue(commandMechanism.intake());
    // driverController.rightBumper().whileTrue(hood.reachGoal(.09774));
    driverController.rightBumper().whileTrue(gameState.shoot());
    driverController.leftBumper().whileTrue(gameState.intake());
    // driverController.leftBumper().whileTrue(Commands.defer(()->drivetrain.toArcWhilePoint(GameData.getHubPose3d().toPose2d(), GameData.getHubPose3d().toPose2d(),2,5,5),Set.of(drivetrain)));
    // driverController.rightBumper().whileTrue(Commands.defer(()->drivetrain.pointWhileDrive(GameData.getHubPose3d().toPose2d(), driverController, 5,1,5,1), Set.of(drivetrain)));
  }

  public RobotMain() {
    drivetrain.configurePathPlanner();
    
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());
    DataLogManager.logNetworkTables(true);
    
    new SequentialAutos(commandMechanism);

    
    autoChooser = AutoBuilder.buildAutoChooser();
    
    createAutos();
    // autoChooser = buildAutoChooser("", (data) -> data);

    // autoChooser.onChange((data)->{
    //   try{
    //     if (AutoBuilder.getAllAutoNames().contains(data.getName())){
    //       PathPlannerAuto auto = new PathPlannerAuto(data.getName());
    //       drivetrain.resetPose(auto.getStartingPose());
    //     }else{
    //       Pose2d autoStart = waitAutos.getStartingPose(data.getName());
    //       if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red){
    //         autoStart = PoseEX.pose180(autoStart);
    //       }
    //       drivetrain.resetPose(autoStart);
    //     }
    //   } catch(Exception e){
        
    //   }
    // });

    

    SmartDashboard.putData("Auto Chooser", autoChooser);

    SmartDashboard.putData(CommandScheduler.getInstance());

    configureBindings();

    //How you might make a choreo only path
    autoChooser.addOption("ChoreoPath", ChoreoEX.getChoreoGroupPath(true,new String[]{"shootPreAmp","intake4","shoot4M","intake5","shoot5M","intake6","shoot6M","intake7","shoot7M"}));
  
    basePublisher.accept(new Pose2d());
  }

  public void configureAutonomousCommands() {
    NamedCommands.registerCommand("intake", commandMechanism.intake().alongWith(Commands.print("HIII")));
    NamedCommands.registerCommand("shoot", commandMechanism.intake());
  }

  public void createAutos(){
    autoChooser.addOption("First", Commands.none());
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  @Deprecated
  public static SendableChooser<Command> buildAutoChooser(
      String defaultAutoName,
      Function<Stream<PathPlannerAuto>, Stream<PathPlannerAuto>> optionsModifier) {
    if (!AutoBuilder.isConfigured()) {
      throw new RuntimeException(
          "AutoBuilder was not configured before attempting to build an auto chooser");
    }

    SendableChooser<Command> chooser = new SendableChooser<>();
    List<String> autoNames = AutoBuilder.getAllAutoNames();

    PathPlannerAuto defaultOption = null;
    List<PathPlannerAuto> options = new ArrayList<>();

    for (String autoName : autoNames) {
      PathPlannerAuto auto;
      try {
        auto = new PathPlannerAuto(autoName);
      } catch (Exception e) {
        auto = new PathPlannerAuto(Commands.none());
      }
      
      if (!defaultAutoName.isEmpty() && defaultAutoName.equals(autoName)) {
        defaultOption = auto;
      } else {
        options.add(auto);
      }
    }

    if (defaultOption == null) {
      chooser.setDefaultOption("None", Commands.none());
    } else {
      chooser.setDefaultOption(defaultOption.getName(), defaultOption);
      chooser.addOption("None", Commands.none());
    }

    optionsModifier
        .apply(options.stream())
        .forEach(auto -> chooser.addOption(auto.getName(), auto));

    return chooser;
  }
}