// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Function;
import java.util.stream.Stream;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
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
import frc.robot.commands.BranchAutos;
import frc.robot.commands.BranchAutos.BranchInstruction;
import frc.robot.commands.BranchAutos.Positions;
import frc.robot.constants.GameData;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandMechanism;
import frc.robot.subsystems.TurretMechanism.Hood;
import frc.robot.subsystems.TurretMechanism.Shooter;
import frc.robot.subsystems.TurretMechanism.Turret;
import frc.robot.subsystems.intakeMechanism.Arm;
import frc.robot.subsystems.intakeMechanism.Indexer;
import frc.robot.subsystems.intakeMechanism.Intake;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.swerve.swerveHelpers.Telemetry;
import frc.robot.util.ChoreoEX;


public class RobotMain extends RobotContainer {

  private SendableChooser<Command> autoChooser;

  private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps;
  private double MaxAngularRate = TunerConstants.MAX_ANGULAR_RATE;

  private double speedPercent = 1;
  private double rotationPercent = 1;


  private final CommandXboxController driverController = new CommandXboxController(0);
  
  private final SwerveDrive drivetrain = new SwerveDrive();

  private final Arm arm = new Arm();
  private final Indexer indexer = new Indexer();
  private final Intake intake = new Intake();
  private final Hood hood = new Hood();
  private final Shooter shooter = new Shooter();
  private final Turret turret = new Turret();

  private final CommandMechanism commandMechanism = new CommandMechanism(arm,intake,indexer,turret,hood,shooter,drivetrain);
  private final BranchAutos branchAutos = new BranchAutos(commandMechanism);

  // private final ObjectDetection objectDetection = new ObjectDetection("Test",
  //   new Transform3d(new Translation3d(0,-.101, .522), new Rotation3d(0, Units.degreesToRadians(-20), Units.degreesToRadians(0))), ()->drivetrain.getPose());
  
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(0).withRotationalDeadband(0)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); 

  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

  private final Telemetry logger = new Telemetry(MaxSpeed);

  private void configureBindings() {
    drivetrain.resetPose(new Pose2d(7,5,Rotation2d.fromDegrees(180)));
  
    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(() -> drive.withVelocityX(-driverController.getLeftY() * speedPercent * MaxSpeed)
            .withVelocityY(-driverController.getLeftX() * speedPercent * MaxSpeed)
            .withRotationalRate(-driverController.getRightX() * rotationPercent * MaxAngularRate)
    ));
    
    drivetrain.getDriveIO().registerTelemetry((log)->logger.telemeterize(log));
    
    driverController.y().onTrue(Commands.runOnce(() -> drivetrain.seedFieldRelative(drivetrain.getPose().getRotation())));

    new Trigger(()->DriverStation.isTeleop()).onTrue(Commands.runOnce(()->{
      if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red){
        drivetrain.seedFieldRelative(Rotation2d.fromDegrees(180));
      }else{
        drivetrain.seedFieldRelative(Rotation2d.fromDegrees(0));
      }
    }));

    StructPublisher<Pose2d> armPublisher = NetworkTableInstance.getDefault()
    .getStructTopic("pose2d", Pose2d.struct).publish();
    armPublisher.set(new Pose2d());
    // arm.setDefaultCommand(arm.reachGoal(0));
    // turret.setDefaultCommand(turret.reachGoal(0));
    // hood.setDefaultCommand(hood.reachGoal(0));
    // driverController.a().whileTrue(arm.reachGoal(.25));
    // driverController.b().whileTrue(turret.reachGoal(.5));
    // driverController.x().whileTrue(hood.reachGoal(.5));
    driverController.leftBumper().whileTrue(commandMechanism.intake());
    driverController.rightTrigger().whileTrue(commandMechanism.scoreWhileMoving(GameData.scorePose3d));
    driverController.rightBumper().whileTrue(commandMechanism.autoScore(GameData.scorePose3d));

    // driverController.y().whileTrue(commandMechanism.testScore(2,0));
    // driverController.a().whileTrue(commandMechanism.testScore(2.5,0));
    // driverController.b().whileTrue(commandMechanism.testScore(3,0));
    // driverController.x().whileTrue(commandMechanism.testScore(3.5,0));

    driverController.x().onTrue(Commands.runOnce(()->{
      drivetrain.resetPose(new Pose2d(-.4,0,new Rotation2d()));
    }));
    // driverController.y().whileTrue(commandMechanism.testShoot(40,0.25));
    // driverController.a().whileTrue(commandMechanism.testShoot(45,0.19));
    // driverController.b().whileTrue(commandMechanism.testShoot(50,0.19));
    // driverController.x().whileTrue(commandMechanism.testShoot(15,0.25));
  }

  public RobotMain() {
    drivetrain.configurePathPlanner();
    
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());
    DataLogManager.logNetworkTables(true);
    
    configureAutonomousCommands();

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
  }

  public void configureAutonomousCommands() {
    
  }

  public void createAutos(){
    autoChooser.addOption("First"
      ,branchAutos.auto("First", new Pose2d(GameData.fieldSizeX/2 - .5, GameData.fieldSizeY/2 + 1, new Rotation2d())
      , "NONE"
      , new BranchInstruction(Positions.BeginUp,Positions.Cargo3),new BranchInstruction(Positions.Cargo3,Positions.Score3)
      , new BranchInstruction(Positions.Score3,Positions.Cargo2),new BranchInstruction(Positions.Cargo2,Positions.Cargo1)
      , new BranchInstruction(Positions.Cargo1,Positions.Score1),new BranchInstruction(Positions.Score1,Positions.Feeder)
      ,new BranchInstruction(Positions.Feeder,Positions.ScoreF)));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

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