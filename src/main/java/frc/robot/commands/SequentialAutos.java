package frc.robot.commands;

import java.util.function.BiConsumer;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.DriveFeedforwards;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.GameData;
import frc.robot.subsystems.CommandMechanism;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.util.PoseEX;

public class SequentialAutos {

    private final CommandMechanism commandMechanism;
    private final SwerveDrive swerveDrive;
    private BiConsumer<ChassisSpeeds, DriveFeedforwards> defaultAuto;

    /**
     * create this before the autoChooser is built
     * @param commandMechanism
     */
    public SequentialAutos(CommandMechanism commandMechanism){
        this.commandMechanism = commandMechanism;
        this.swerveDrive = commandMechanism.swerveDrive;

        defaultAuto = commandMechanism.swerveDrive.getAutoConsumer();

        NamedCommands.registerCommand("shoot", shoot());
        NamedCommands.registerCommand("shootStatic", shootStatic());
        NamedCommands.registerCommand("passLeft", passLeft());
        NamedCommands.registerCommand("passRight", passRight());
        NamedCommands.registerCommand("intake", intake());
        NamedCommands.registerCommand("stopShooting", stopShooting());
        NamedCommands.registerCommand("stopIntake", stopIntake());
    }

    public BiConsumer<ChassisSpeeds, DriveFeedforwards> pointToRotationConsumer(Supplier<Rotation2d> pointTo){
        BiConsumer<ChassisSpeeds, DriveFeedforwards> pointToAuto = (speeds, feedForward)->{
            double omegaRadiansPerSecond = swerveDrive.poseNavigation.calculateTowardRotation(pointTo.get(), commandMechanism.swerveDrive.getPose(), 5).omegaRadiansPerSecond;
            ChassisSpeeds s = new ChassisSpeeds(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, omegaRadiansPerSecond);
            swerveDrive.setControl(swerveDrive.robotCentricDrive.withSpeeds(s.times(1)));
        };
        return pointToAuto;
    }

    public BiConsumer<ChassisSpeeds, DriveFeedforwards> pointToPoseConsumer(Supplier<Pose2d> pointTo){
        BiConsumer<ChassisSpeeds, DriveFeedforwards> pointToAuto = (speeds, feedForward)->{
            double omegaRadiansPerSecond = swerveDrive.poseNavigation.calculateTowardRotation(pointTo.get(), commandMechanism.swerveDrive.getPose(), 5).omegaRadiansPerSecond;
            ChassisSpeeds s = new ChassisSpeeds(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, omegaRadiansPerSecond);
            swerveDrive.setControl(swerveDrive.robotCentricDrive.withSpeeds(s.times(1)));
        };
        return pointToAuto;
    }

    public BiConsumer<ChassisSpeeds, DriveFeedforwards> getToConsumer(Supplier<Pose2d> getTo){
        BiConsumer<ChassisSpeeds, DriveFeedforwards> getToAuto = (speeds, feedForward)->{
            ChassisSpeeds s = swerveDrive.poseNavigation.calculateTowardPose(getTo.get(), swerveDrive.getPose(),5,5);
            swerveDrive.setControl(swerveDrive.fieldCentricDrive.withSpeeds(s.times(1)));
        };
        return getToAuto;
    }

    public Command shoot(){
        BiConsumer<ChassisSpeeds, DriveFeedforwards> shootingAuto = pointToRotationConsumer(()->Rotation2d.fromRotations(commandMechanism.getDynamicScoringData()[2]));
        return wrapCommand(Commands.sequence(
            Commands.runOnce(()->swerveDrive.setAutoConsumer(shootingAuto))
            ,commandMechanism.shootDefault(()->commandMechanism.getDynamicScoringData(),()->commandMechanism.readyToShootHub())));
    }

    public Command shootStatic(){
        return commandMechanism.shootStatic().unless(()->(GameData.isRed.getAsBoolean() ? PoseEX.pose180(commandMechanism.swerveDrive.getPose()) : commandMechanism.swerveDrive.getPose()).getX() > GameData.scorePose2d.getX());
    }

    public Command stopShooting(){
        return commandMechanism.stopShooting();
    }

    public Command stopIntake(){
        return commandMechanism.stopIntake();
    }

    public Command passLeft(){
        BiConsumer<ChassisSpeeds, DriveFeedforwards> passLeftAuto = pointToRotationConsumer(()->Rotation2d.fromRotations(commandMechanism.getDynamicPassingData(true)[2]));
        return wrapCommand(Commands.sequence(
            Commands.runOnce(()->swerveDrive.setAutoConsumer(passLeftAuto))
            ,commandMechanism.shootDefault(()->commandMechanism.getDynamicPassingData(true),()->true)));
    }

    public Command passRight(){
        BiConsumer<ChassisSpeeds, DriveFeedforwards> passRightAuto = pointToRotationConsumer(()->Rotation2d.fromRotations(commandMechanism.getDynamicPassingData(false)[2]));
        return wrapCommand(Commands.sequence(
            Commands.runOnce(()->swerveDrive.setAutoConsumer(passRightAuto))
            ,commandMechanism.shootDefault(()->commandMechanism.getDynamicPassingData(false),()->true)));
    }

    public Command intake(){
        return commandMechanism.intake();
    }

    public Command wrapCommand(Command command){
        return command
            .finallyDo(()->commandMechanism.swerveDrive.setAutoConsumer(defaultAuto));
    }
}
