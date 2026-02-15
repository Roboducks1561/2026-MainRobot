package frc.robot.subsystems;

import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.BooleanPublisher;
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
    private boolean interpolate = false;
    private final double maxAimError = .02;

    protected final Animations animations;

    private final BooleanPublisher aimCorrect = readyToShootRequirements
        .getBooleanTopic("aimCorrect").publish();

    public CommandMechanism(Arm arm, Intake intake, Indexer leftIndexer, Indexer rightIndexer,Shooter leftShooter,Shooter rightShooter, Spindexer spindexer, Hood hood, SwerveDrive swerveDrive){
        super(arm, intake, leftIndexer, rightIndexer, leftShooter, rightShooter, spindexer, hood, swerveDrive);
        scoreMath = new ScoreMath(swerveDrive, fromSwerveBase);

        if (Robot.isSimulation()){
            Bootleg2026.addShooterSimulation(
                ()->fromSwerveBase.plus(new Transform3d(0,0,0,new Rotation3d(0,Units.rotationsToRadians(.25-hoodRotationsToShootRotations(hood.getPosition())),0)))
                    ,()->leftShooter.getVelocity() * 1.07, "Fuel", "Intake");
            Bootleg2026.addShootRequirements("Intake", ()->{
                boolean ready = leftShooter.getVelocity() > 1 && leftIndexer.getVelocity() > 2 && Utils.getCurrentTimeSeconds() > lastLaunchLeft + .15;
                if (ready){
                    lastLaunchLeft = Utils.getCurrentTimeSeconds();
                }
                return ready;
            });

            Bootleg2026.addIntakeSimulation("Intake","Fuel",.5,1,60,new Translation2d(.2,0));
            Bootleg2026.addIntakeRequirements("Intake", ()->Math.abs(arm.getPosition() - armIntakePosition) < .05);
            
            // int[] lastI = new int[]{1};
            // Bootleg2026.hasPiece("Intake",(i)->{
            //     intake.getMotorStrainIO().setValue(lastI[0] < i);
            //     lastI[0] = i;
            //     intake.getDigitalInputIO().setValue(i == 2);
            //     indexer.getDigitalInputIO().setValue(i == 1 || i == 2);
            // });
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
        return Math.abs(dynamicScoringData[2]-swerveDrive.getYaw().getRotations()) < maxAimError;
    }

    public double mpsTorps(double mps){
        return Robot.isSimulation() ? mps : mps*10;
    }

    public double rpsTomps(double rps){
        return Robot.isSimulation() ? rps : rps/10;
    }

    //TODO, likely the conversion of hood rotations to shoo rotations is not linear, so fix when physical robot is available.
    public double hoodRotationsToShootRotations(double rotations){
        return Robot.isSimulation() ? rotations : rotations + HoodConstants.MIN_HOOD_ANGLE_RAD;
    }

    public double shootRotationsToHoodRotations(double rotations){
        return Robot.isSimulation() ? rotations : rotations - HoodConstants.MIN_HOOD_ANGLE_RAD;
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
        return Commands.parallel(intake.reachGoalOnce(0), arm.reachGoalOnce(0));
    }

    public Command shootDefault(Supplier<double[]> dynamicScoringData, BooleanSupplier ready){
        return shootBothContinuous(()->shootRotationsToHoodRotations(dynamicScoringData.get()[0])
            ,()->mpsTorps(dynamicScoringData.get()[1])
            ,()->PoseEX.correctedRotation(dynamicScoringData.get()[2]-swerveDrive.getYaw().getRotations())
            , ready);
    }

    public Command shootStatic() {
        return Commands.parallel(swerveDrive.rotateTo(()->Rotation2d.fromRotations(dynamicScoringData[2]),5).until(()->swerveDrive.withinRotation(Rotation2d.fromRotations(dynamicScoringData[2]), .02)).andThen(swerveDrive.brake())
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
            ,hoodRotationsToShootRotations(Units.radiansToRotations(HoodConstants.MIN_HOOD_ANGLE_RAD)), hoodRotationsToShootRotations(Units.radiansToRotations(HoodConstants.MAX_HOOD_ANGLE_RAD)), rpsTomps(8));

        dynamicPassLeft = scoreMath.dynamicScore(GameData.getPassPose3d(true), interpolate && !Robot.isSimulation()
            ,Units.radiansToRotations(HoodConstants.MIN_HOOD_ANGLE_RAD), hoodRotationsToShootRotations(Units.radiansToRotations(HoodConstants.MAX_HOOD_ANGLE_RAD)), rpsTomps(10));
        dynamicPassRight = scoreMath.dynamicScore(GameData.getPassPose3d(false), interpolate && !Robot.isSimulation()
            ,Units.radiansToRotations(HoodConstants.MIN_HOOD_ANGLE_RAD), hoodRotationsToShootRotations(Units.radiansToRotations(HoodConstants.MAX_HOOD_ANGLE_RAD)), rpsTomps(10));
    
        if (!Robot.isSimulation() && leftShooter.getVelocity() > 1 && leftIndexer.getVelocity() > 2 && Utils.getCurrentTimeSeconds() > lastLaunchLeft + .2){
            lastLaunchLeft = Utils.getCurrentTimeSeconds();
            animations.addFlyingObject(swerveDrive.getPose(), fromSwerveBase.getTranslation(), new Rotation3d(0,Units.rotationsToRadians(.25-hoodRotationsToShootRotations(hood.getPosition())),Units.rotationsToRadians(0)), swerveDrive.getSpeeds(), rpsTomps(leftShooter.getVelocity()));
        }
        if (!Robot.isSimulation() && rightShooter.getVelocity() > 1 && rightIndexer.getVelocity() > 2 && Utils.getCurrentTimeSeconds() > lastLaunchRight + .2){
            lastLaunchRight = Utils.getCurrentTimeSeconds();
            animations.addFlyingObject(swerveDrive.getPose(), fromSwerveBase.getTranslation(), new Rotation3d(0,Units.rotationsToRadians(.25-hoodRotationsToShootRotations(hood.getPosition())),Units.rotationsToRadians(0)), swerveDrive.getSpeeds(), rpsTomps(rightShooter.getVelocity()));
        }

        aimCorrect.accept(readyToShootHub());
    }
}