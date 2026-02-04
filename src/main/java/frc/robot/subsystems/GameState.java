package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.GameData;
import frc.robot.util.PoseEX;

public class GameState {
    private final Notifier notifier;
    private boolean shooting = true;
    private boolean hiding = false;
    private boolean passingLeft = false;

    private boolean wonAuto = true;

    //TimeFrames (Auto: 0-20) (TransitionShift: 20-30) (Shift 1: 30-55) (Shift 2: 55-80) (Shift 3: 80-105) (Shift 4: 105-130) (Endgame: 130-160)
    private int gameState = 0;
    private int[] gameTimes = new int[]{10,35,60,85,110};

    private final CommandMechanism commandMechanism;
    private final CommandXboxController driverController;


    public GameState(CommandMechanism commandMechanism, CommandXboxController driverController){
        this.commandMechanism = commandMechanism;
        this.driverController = driverController;

        notifier = new Notifier(this :: periodic);
        notifier.setName("GameLogic Periodic");
        notifier.startPeriodic(.02);
        Runtime.getRuntime().addShutdownHook(new Thread(notifier::close));

        SendableChooser<Boolean> wonAutoSendableChooser = new SendableChooser<>();

        wonAutoSendableChooser.addOption("Lost", false);
        wonAutoSendableChooser.setDefaultOption("Won", true);

        SmartDashboard.putData("WonAuto?", wonAutoSendableChooser);

        wonAutoSendableChooser.onChange((value)->{
            wonAuto = value;
        });

        matchTime = Timer.getFPGATimestamp();
        gameState = 0;

        new Trigger(()->DriverStation.isTeleop()).onTrue(Commands.runOnce(()->{
            matchTime = Timer.getFPGATimestamp();
            gameState = 0;
        }));
    }

    public boolean shootingPeriod(){
        return gameState < 1 || gameState == 5 || (wonAuto ? (gameState == 2 || gameState == 4) : (gameState == 1 || gameState == 3));
    }

    public Command shoot(){
        return shooting().until(()->hiding);
    }

    private Command shooting(){
        return Commands.either(commandMechanism.shootDynamic(()->-driverController.getLeftY(), ()->-driverController.getLeftX()).unless(()->!shootingPeriod()).until(()->!shootingPeriod())
            ,commandMechanism.passDynamic(()->passingLeft,()->-driverController.getLeftY(), ()->-driverController.getLeftX()), ()->shooting);
    }

    public Command intake(){
        return commandMechanism.intake();
    }

    double matchTime = -200;
    public void periodic(){
        Pose2d swervePose = commandMechanism.swerveDrive.getPose();
        boolean isRed = GameData.isRed.getAsBoolean();
        Pose2d biasedSwervePose = isRed ? PoseEX.pose180(swervePose) : swervePose;
        if (gameState < 5 && Timer.getFPGATimestamp() - matchTime > gameTimes[gameState]){
            gameState++;
        }

        hiding = PoseEX.isNear(swervePose, GameData.dangerousPoses,1.5);

        shooting = biasedSwervePose.getX() < GameData.scorePose2d.getX();

        passingLeft = biasedSwervePose.getY() > GameData.scorePose2d.getY();
    }
}
