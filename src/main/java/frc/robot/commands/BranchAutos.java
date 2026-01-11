package frc.robot.commands;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.constants.GameData;
import frc.robot.subsystems.CommandMechanism;
import frc.robot.util.DynamicObstacle;
import frc.robot.util.PoseEX;

public class BranchAutos {
    //meters
    double minScoreDistance = 5;
    public enum Positions{
        Score1(GameData.scorePose2d, "S1"),
        Score2(GameData.scorePose2d, "S2"),
        Score3(GameData.scorePose2d, "S3"),
        Score4(GameData.scorePose2d, "S4"),
        Score5(GameData.scorePose2d, "S5"),
        Score6(GameData.scorePose2d, "S6"),
        ScoreF(GameData.scorePose2d, "SF"),
        BeginUp(new Pose2d(GameData.fieldSizeX/2 - 1.5,GameData.fieldSizeY/2 + 1,new Rotation2d()),"BU"),
        BeginDown(new Pose2d(GameData.fieldSizeX/2 - 1.5,GameData.fieldSizeY/2 - 1,new Rotation2d()),"BD"),
        Cargo1(GameData.getCargoPose(1,false),"C1"),
        Cargo2(GameData.getCargoPose(2,false),"C2"),
        Cargo3(GameData.getCargoPose(3,false),"C3"),
        Cargo4(GameData.getCargoPose(4,false),"C4"),
        Cargo5(GameData.getCargoPose(5,false),"C5"),
        Cargo6(GameData.getCargoPose(6,false),"C6"),
        Feeder(GameData.getFeederPose(false),"F");

        public final Pose2d pose;
        public final String identifier;
        Positions(Pose2d pose, String identifier){
            this.pose = pose;
            this.identifier = identifier;
        }
    }

    public static class BranchInstruction{
        public final String to;
        public final String from;

        public final Pose2d fromPose;
        public final Pose2d toPose;

        public BranchInstruction(Positions from, Positions to){
            this.from = from.identifier;
            this.to = to.identifier;
            this.fromPose = from.pose;
            this.toPose = to.pose;
        }
    }

    private final CommandMechanism commandMechanism;

    public BranchAutos(CommandMechanism commandMechanism){
        this.commandMechanism = commandMechanism;
    }

    public Command auto(String nameOfAuto, Pose2d beginPose, String dynamicObstacles, BranchInstruction... positions){

        boolean avoidObstacles = false;
        if (dynamicObstacles.length() == 0){
            DynamicObstacle.clearDynamicObstacles(beginPose.getTranslation());
        }else{
            DynamicObstacle.setDynamicObstacles(dynamicObstacles,beginPose.getTranslation());
            avoidObstacles = true;
        }
        SequentialCommandGroup commandGroup = new SequentialCommandGroup();

        Pose2d startPose = beginPose;
        

        try {
            //for odom reset
            PathPlannerPath path = PathPlannerPath.fromChoreoTrajectory(positions[0].from+positions[0].to);
            startPose = new Pose2d(path.getAllPathPoints().get(0).position,path.getAllPathPoints().get(0).rotationTarget.rotation());
        } catch (Exception e) {
            System.out.println(positions[0].from+positions[0].to+ " does not exist");
        }

        for (int i = 0; i < positions.length; i++){
            commandGroup.addCommands(tryPath(positions[i], avoidObstacles));
        }

        return AutoBuilder.resetOdom(startPose)
            .andThen(commandGroup).withName(nameOfAuto);
    }

    private Command tryPath(BranchInstruction instruction, boolean avoidObstacles){

        if (avoidObstacles){
            if (instruction.to.startsWith("S")){
                return Commands.defer(()->autoScore(
                    commandMechanism.swerveDrive.toPoseAndPoint(PoseEX.getInbetweenPose2d(GameData.scorePose2d, commandMechanism.swerveDrive.getPose(),minScoreDistance)
                    , GameData.scorePose2d
                    ,minScoreDistance+.5
                    ,5
                    ,Math.PI*2)), commandMechanism.subsystems);
            }
            return Commands.defer(()->autoIntake(
                    commandMechanism.swerveDrive.toPose(PoseEX.getInbetweenPose2d(commandMechanism.swerveDrive.getPose(), instruction.toPose, PoseEX.getPoseAngle(commandMechanism.swerveDrive.getPose(), instruction.toPose).rotateBy(Rotation2d.k180deg),.2)
                    ,1
                    ,5
                    ,Math.PI*2)
                ,instruction.toPose, instruction.to.startsWith("F")), commandMechanism.subsystems);

            // return Commands.defer(()->autoIntake(
            //     commandMechanism.swerveDrive.toPose(new Pose2d(instruction.toPose.getX(), instruction.toPose.getY(), PoseEX.getPoseAngle(commandMechanism.swerveDrive.getPose(), instruction.toPose).rotateBy(Rotation2d.k180deg))
            //     ,1
            //     ,5
            //     ,Math.PI*2)
            // ,instruction.toPose, false), commandMechanism.subsystems);

        }
        if (instruction.to.startsWith("S")){
            return Commands.defer(()->{
                try {
                    return autoScore(AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory(instruction.from+instruction.to)));
                } catch (FileVersionException | IOException | ParseException e) {
                    System.out.println(instruction.from+instruction.to +" DNE");
                    return autoScore(
                        commandMechanism.swerveDrive.toPoseAndPoint(PoseEX.getInbetweenPose2d(GameData.scorePose2d, commandMechanism.swerveDrive.getPose(),minScoreDistance)
                        , GameData.scorePose2d
                        ,minScoreDistance+.5
                        ,5
                        ,Math.PI*2));
                }
            },commandMechanism.subsystems);
        }
        return Commands.defer(()->{
            try {
                return autoIntake(AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory(instruction.from+instruction.to))
                ,instruction.toPose, false);
            } catch (FileVersionException | IOException | ParseException e) {
                return autoIntake(
                    commandMechanism.swerveDrive.toPose(new Pose2d(instruction.toPose.getX(), instruction.toPose.getY(), PoseEX.getPoseAngle(commandMechanism.swerveDrive.getPose(), instruction.toPose).rotateBy(Rotation2d.k180deg))
                    ,1
                    ,5
                    ,Math.PI*2)
                ,instruction.toPose, false);
            }
        }, commandMechanism.subsystems);
    }

    private Command autoIntake(Command path, Pose2d pose, boolean two){
        return Commands.race(path
        , Commands.waitUntil(()->commandMechanism.swerveDrive.withinCoords(pose, 2, Math.PI*2)).andThen(commandMechanism.intake().withTimeout(2))
        , Commands.waitUntil(()->commandMechanism.intake.intaking() || (commandMechanism.intake.hasPiece() && commandMechanism.indexer.hasPiece())).andThen(Commands.waitSeconds(commandMechanism.intakeTime))
            .andThen(Commands.either(
                Commands.waitUntil(()->commandMechanism.intake.intaking() || (commandMechanism.intake.hasPiece() && commandMechanism.indexer.hasPiece())).andThen(Commands.waitSeconds(commandMechanism.intakeTime))
                , Commands.none()
                ,()->two))
        ).andThen(commandMechanism.reset());
    }

    private Command autoScore(Command path){
        return path.until(()->PoseEX.getDistanceFromPoseMeters(commandMechanism.swerveDrive.getPose(), GameData.scorePose2d) < minScoreDistance)
            .andThen(commandMechanism.autoScore(GameData.scorePose3d)).andThen(commandMechanism.reset());
    }
}
