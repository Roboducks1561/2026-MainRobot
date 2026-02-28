package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.constants.GameData;
import frc.robot.constants.HoodConstants;
import frc.robot.subsystems.TurretMechanism.Hood;
import frc.robot.subsystems.TurretMechanism.Indexer;
import frc.robot.subsystems.TurretMechanism.Shooter;
import frc.robot.subsystems.intakeMechanism.Arm;
import frc.robot.subsystems.intakeMechanism.Intake;
import frc.robot.subsystems.intakeMechanism.Spindexer;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.util.PoseEX;
import frc.robot.util.mapleSim.Animations;
import frc.robot.util.mapleSim.Bootleg2026;

public class CommandMechanism extends BaseMechanism{
    private final Notifier notifier;

    private final ScoreMath scoreMath;
    private double[] dynamicScoringData = new double[]{0,0,0};
    private double[] dynamicPassLeft = new double[]{0,0,0};
    private double[] dynamicPassRight = new double[]{0,0,0};
    private boolean interpolate = true;
    private final double maxAimError = .03;

    protected final Animations animations;

    private final BooleanPublisher aimCorrect = readyToShootRequirements
        .getBooleanTopic("aimCorrect").publish();

    private final DoublePublisher distance = readyToShootRequirements
        .getDoubleTopic("distance").publish();

    public CommandMechanism(Arm arm, Intake intake, Indexer leftIndexer, Indexer rightIndexer,Shooter leftShooter,Shooter rightShooter, Spindexer spindexer, Hood hood, SwerveDrive swerveDrive){
        super(arm, intake, leftIndexer, rightIndexer, leftShooter, rightShooter, spindexer, hood, swerveDrive);
        scoreMath = new ScoreMath(swerveDrive, fromSwerveBase);

        if (Robot.isSimulation()){
            Bootleg2026.addShooterSimulation(
                ()->fromSwerveBase.plus(new Transform3d(0,0,0,new Rotation3d(0,Units.rotationsToRadians(.25-hoodRotationsToShootRotations(hood.getPosition())),0)))
                    ,()->rpsTomps(leftShooter.getVelocity()) * 1.07, "Fuel", "Intake");
            Bootleg2026.addShootRequirements("Intake", ()->{
                boolean ready = leftShooter.getVelocity() > 1 && leftIndexer.getVelocity() > 2 && Utils.getCurrentTimeSeconds() > lastLaunchLeft + .15;
                if (ready){
                    lastLaunchLeft = Utils.getCurrentTimeSeconds();
                }
                return ready;
            });

            Bootleg2026.addIntakeSimulation("Intake","Fuel",.5,1,60,new Translation2d(.2,0));
            Bootleg2026.addIntakeRequirements("Intake", ()->Math.abs(arm.getPosition() - armIntakePosition) < .05);
            
            Bootleg2026.hasPiece("Intake",(i)->{
                leftIndexer.getDigitalInputIO().setValue(i>0);
                rightIndexer.getDigitalInputIO().setValue(i>0);
            });
        }

        notifier = new Notifier(this :: commandPeriodic);
        notifier.setName("CommandMechanism Periodic");
        notifier.startPeriodic(.02);
        Runtime.getRuntime().addShutdownHook(new Thread(notifier::close));

        animations = new Animations();
    }

    /**
     * @returns {Shoot rotations, velocity meters per second, target direction rotations}
     */
    public double[] getDynamicScoringData(){
        return dynamicScoringData;
    }

    /**
     * @returns {Shoot rotations, velocity meters per second, target direction rotations}
     */
    public double[] getDynamicPassingData(boolean left){
        return left ? dynamicPassLeft: dynamicPassRight;
    }

    public boolean readyToShootHub(){
        return Math.abs(PoseEX.correctedRotation(dynamicScoringData[2]-swerveDrive.getPose().getRotation().getRotations())) < maxAimError;//Math.abs(PoseEX.correctedRotation(dynamicScoringData[2]-swerveDrive.getPose().getRotation().getRotations())) < maxAimError || Math.abs(PoseEX.correctedRotation(dynamicScoringData[2]-swerveDrive.getPose().getRotation().getRotations())) > .5 - maxAimError;//Robot.isSimulation() ? Math.abs(PoseEX.correctedRotation(dynamicScoringData[2]-swerveDrive.getPose().getRotation().getRotations())) < maxAimError : Math.abs(PoseEX.correctedRotation(dynamicScoringData[2]-swerveDrive.getPose().getRotation().getRotations())) > .5 - maxAimError;
    }

    public double mpsTorps(double mps){ 
        return Robot.isSimulation() ? mps:mps;//mps*11 : mps*11;
    }

    public double rpsTomps(double rps){
        return Robot.isSimulation() ? rps:rps;//rps/11 : rps/11;
    }

    //TODO, likely the conversion of hood rotations to shoo rotations is not linear, so fix when physical robot is available.
    public double hoodRotationsToShootRotations(double rotations){
        return Robot.isSimulation() ? rotations:rotations;//rotations*1.2 + .02 : rotations*1.2 + .02;
    }
    // ax + b = y
    // ay + b = x

    // (x-b)/a = y

    public double shootRotationsToHoodRotations(double rotations){
        return Robot.isSimulation() ? rotations:rotations;//(rotations - .02)/1.2 : (rotations - .02)/1.2;
    }

    public Command stopShooting(){
        return Commands.parallel(leftShooter.reachGoalOnce(0),
            rightShooter.reachGoalOnce(0),
            hood.reachGoalOnce(0), 
            spindexer.reachGoalOnce(0), 
            leftIndexer.reachGoalOnce(0),
            rightIndexer.reachGoalOnce(0));
    }

    public Command stopIntake(){
        return intakeRollers().withTimeout(.5).andThen(Commands.parallel(intake.reachGoal(()->((int)Utils.getCurrentTimeSeconds()*5)%4 == 0 ? -intakeSpeed : intakeSpeed), arm.reachGoalOnce(0).until(()->arm.getPosition()-.05 < 0).andThen(arm.setVoltage(-1.3))));
    }

    public Command shootDefault(Supplier<double[]> dynamicScoringData, BooleanSupplier ready){
        return shootBothContinuous(()->shootRotationsToHoodRotations(dynamicScoringData.get()[0])
            ,()->mpsTorps(dynamicScoringData.get()[1])
            ,()->PoseEX.correctedRotation(dynamicScoringData.get()[2]-swerveDrive.getPose().getRotation().getRotations())
            , ready);
    }

    public Command shootStatic() {
        return Commands.parallel(swerveDrive.rotateTo(()->Rotation2d.fromRotations(dynamicScoringData[2]),5)//.until(()->swerveDrive.withinRotation(Rotation2d.fromRotations(dynamicScoringData[2]), .01)).andThen(swerveDrive.brake())
            ,shootDefault(()->dynamicScoringData, ()->readyToShootHub()));
    }

    public Command shootDynamic(DoubleSupplier vx, DoubleSupplier vy) {
        return shootDefault(()->dynamicScoringData, ()->readyToShootHub())
            .alongWith(swerveDrive.pointWhileDrive(()->Rotation2d.fromRotations(dynamicScoringData[2]), vx,vy, 5,1,5,1));    
    }

    public Command passStatic(boolean left) {
        return Commands.parallel(swerveDrive.rotateTo(()->Rotation2d.fromRotations(left ? dynamicPassLeft[2] : dynamicPassRight[2]),5)
            ,shootDefault(()-> left ? dynamicPassLeft : dynamicPassRight, ()->true));
    }

    public Command passDynamic(BooleanSupplier left, DoubleSupplier vx, DoubleSupplier vy) {
        return shootDefault(()-> left.getAsBoolean() ? dynamicPassLeft : dynamicPassRight, ()->true)
            .alongWith(swerveDrive.pointWhileDrive(()->Rotation2d.fromRotations(left.getAsBoolean() ? dynamicPassLeft[2] : dynamicPassRight[2]), vx, vy, 5,1,5,1));
    }

    public Command passDynamic(boolean left, DoubleSupplier vx, DoubleSupplier vy) {
        return passDynamic(()->left, vx, vy);
    }

    double lastLaunchLeft = 0;
    double lastLaunchRight = 0;
    public void commandPeriodic(){
        dynamicScoringData = scoreMath.dynamicScore(GameData.getHubPose3d(), interpolate && !Robot.isSimulation()
            ,hoodRotationsToShootRotations(Units.radiansToRotations(HoodConstants.MIN_HOOD_ANGLE_RAD)), hoodRotationsToShootRotations(Units.radiansToRotations(HoodConstants.MAX_HOOD_ANGLE_RAD)), rpsTomps(100));

        dynamicPassLeft = scoreMath.dynamicScore(GameData.getPassPose3d(true), interpolate && !Robot.isSimulation()
            ,Units.radiansToRotations(HoodConstants.MIN_HOOD_ANGLE_RAD), hoodRotationsToShootRotations(Units.radiansToRotations(HoodConstants.MAX_HOOD_ANGLE_RAD)), rpsTomps(100));
        dynamicPassRight = scoreMath.dynamicScore(GameData.getPassPose3d(false), interpolate && !Robot.isSimulation()
            ,Units.radiansToRotations(HoodConstants.MIN_HOOD_ANGLE_RAD), hoodRotationsToShootRotations(Units.radiansToRotations(HoodConstants.MAX_HOOD_ANGLE_RAD)), rpsTomps(100));
        // System.out.println(PoseEX.correctedRotation(dynamicScoringData[2]-swerveDrive.getPose().getRotation().getRotations()));
        if (interpolate){
            dynamicScoringData = new double[]{hoodRotationsToShootRotations(dynamicScoringData[0]), rpsTomps(dynamicScoringData[1]), dynamicScoringData[2]};
            dynamicPassLeft = new double[]{hoodRotationsToShootRotations(dynamicPassLeft[0]), rpsTomps(dynamicPassLeft[1]), dynamicPassLeft[2]};
            dynamicPassRight = new double[]{hoodRotationsToShootRotations(dynamicPassRight[0]), rpsTomps(dynamicPassRight[1]), dynamicPassRight[2]};
        }
        if (!Robot.isSimulation() && leftShooter.getVelocity() > 1 && leftIndexer.getVelocity() > 2 && Utils.getCurrentTimeSeconds() > lastLaunchLeft + .2){
            lastLaunchLeft = Utils.getCurrentTimeSeconds();
            animations.addFlyingObject(swerveDrive.getPose(), fromSwerveBase.getTranslation(), new Rotation3d(0,Units.rotationsToRadians(.25-hoodRotationsToShootRotations(hood.getPosition())),Units.rotationsToRadians(0)), swerveDrive.getSpeeds(), rpsTomps(leftShooter.getVelocity()));
        }
        if (!Robot.isSimulation() && rightShooter.getVelocity() > 1 && rightIndexer.getVelocity() > 2 && Utils.getCurrentTimeSeconds() > lastLaunchRight + .2){
            lastLaunchRight = Utils.getCurrentTimeSeconds();
            animations.addFlyingObject(swerveDrive.getPose(), fromSwerveBase.getTranslation(), new Rotation3d(0,Units.rotationsToRadians(.25-hoodRotationsToShootRotations(hood.getPosition())),Units.rotationsToRadians(0)), swerveDrive.getSpeeds(), rpsTomps(rightShooter.getVelocity()));
        }
        Pose2d turretPose = new Pose3d(swerveDrive.getPose()).transformBy(fromSwerveBase).toPose2d();
        aimCorrect.accept(readyToShootHub());
        distance.accept(PoseEX.getDistanceFromPoseMeters(turretPose, GameData.getHubPose2d()));
    }
}