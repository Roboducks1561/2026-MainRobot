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
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
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

    private final double shootAngle = .055;

    protected final Animations animations;

    NetworkTable table = NetworkTableInstance.getDefault().getTable("Tunables").getSubTable("Interpolation");
    DoubleEntry entry = table.getDoubleTopic("multiplier").getEntry(1);

    private final BooleanPublisher aimCorrect = readyToShootRequirements
        .getBooleanTopic("aimCorrect").publish();

    private final DoublePublisher distance = readyToShootRequirements
        .getDoubleTopic("distance").publish();

    public CommandMechanism(Arm arm, Intake intake, Indexer leftIndexer, Shooter shooter, Spindexer spindexer, Hood hood, SwerveDrive swerveDrive){
        super(arm, intake, leftIndexer, shooter, spindexer, hood, swerveDrive);
        scoreMath = new ScoreMath(swerveDrive, fromSwerveBase);

        if (Robot.isSimulation()){
            Bootleg2026.addShooterSimulation(
                ()->fromSwerveBase.plus(new Transform3d(0,0,0,new Rotation3d(0,Units.rotationsToRadians(.25-shootAngle),0)))
                    ,()->rpsTomps(shooter.getVelocity()), "Fuel", "Intake");
            Bootleg2026.addShootRequirements("Intake", ()->{
                boolean ready = shooter.getVelocity() > 1 && leftIndexer.getVelocity() > 2 && Utils.getCurrentTimeSeconds() > lastLaunchLeft + .15;
                if (ready){
                    lastLaunchLeft = Utils.getCurrentTimeSeconds();
                }
                return ready;
            });

            Bootleg2026.addIntakeSimulation("Intake","Fuel",.5,1,60,new Translation2d(.2,0));
            Bootleg2026.addIntakeRequirements("Intake", ()->Math.abs(arm.getPosition() - armIntakePosition) < .05);
            
            // Bootleg2026.hasPiece("Intake",(i)->{
            //     leftIndexer.getDigitalInputIO().setValue(i>0);
            //     rightIndexer.getDigitalInputIO().setValue(i>0);
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
        return Math.abs(PoseEX.correctedRotation(dynamicScoringData[2]-swerveDrive.getPose().getRotation().getRotations())) < maxAimError;//Math.abs(PoseEX.correctedRotation(dynamicScoringData[2]-swerveDrive.getPose().getRotation().getRotations())) < maxAimError || Math.abs(PoseEX.correctedRotation(dynamicScoringData[2]-swerveDrive.getPose().getRotation().getRotations())) > .5 - maxAimError;//Robot.isSimulation() ? Math.abs(PoseEX.correctedRotation(dynamicScoringData[2]-swerveDrive.getPose().getRotation().getRotations())) < maxAimError : Math.abs(PoseEX.correctedRotation(dynamicScoringData[2]-swerveDrive.getPose().getRotation().getRotations())) > .5 - maxAimError;
    }

    public double rpsTomps(double rps){
        return Robot.isSimulation() ? rps/6.8 : rps/6.8;
    }

    public double mpsToRps(double rps){
        return Robot.isSimulation() ? rps*6.8 : rps*6.8;
    }

    public Command shootDefault(Supplier<double[]> dynamicScoringData, BooleanSupplier ready){
        return shootContinuous(()->dynamicScoringData.get()[0]
            ,()->dynamicScoringData.get()[1]
            ,()->PoseEX.correctedRotation(dynamicScoringData.get()[2]-swerveDrive.getPose().getRotation().getRotations())
            , ready);
    }

    public Command shootStatic() {
        return Commands.parallel(swerveDrive.rotateTo(()->Rotation2d.fromRotations(dynamicScoringData[2]),5)//.until(()->swerveDrive.withinRotation(Rotation2d.fromRotations(dynamicScoringData[2]), .02) && swerveDrive.getSpeeds().omegaRadiansPerSecond < .05).andThen(swerveDrive.brake())
            ,shootDefault(()->dynamicScoringData, ()->readyToShootHub()));
    }

    public Command shootDynamic(DoubleSupplier vx, DoubleSupplier vy) {
        return shootDefault(()->dynamicScoringData, ()->readyToShootHub())
            .alongWith(swerveDrive.pointWhileDrive(()->Rotation2d.fromRotations(dynamicScoringData[2]), vx,vy, 5,1,5,1));    
    }

    // public Command passStatic(boolean left) {
    //     return Commands.parallel(Commands.deadline(swerveDrive.rotateTo(()->Rotation2d.fromRotations(left ? dynamicPassLeft[2] : dynamicPassRight[2]),5))
    //         ,shootContinuous(()->0,()->60,()->0, ()->true));
    // }
    
    public Command passStatic(boolean left) {
        return Commands.parallel(Commands.deadline(
                Commands.waitSeconds(100)
                .until(()->swerveDrive.withinRotation(Rotation2d.fromRotations(dynamicScoringData[2]), .02) && swerveDrive.getSpeeds().omegaRadiansPerSecond < .05),
                swerveDrive.rotateTo(()->Rotation2d.fromRotations(left ? dynamicPassLeft[2] : dynamicPassRight[2]),5))
                .andThen(swerveDrive.brake())
            ,shootContinuous(()->0,()->60,()->0, ()->true));
    }


    public Command passDynamic(BooleanSupplier left, DoubleSupplier vx, DoubleSupplier vy) {
        return shootContinuous(()->0,()->60,()->0, ()->true)
            .alongWith(swerveDrive.pointWhileDrive(()->Rotation2d.fromRotations(left.getAsBoolean() ? dynamicPassLeft[2] : dynamicPassRight[2]), vx, vy, 5,1,5,1));
    }

    public Command passDynamic(boolean left, DoubleSupplier vx, DoubleSupplier vy) {
        return passDynamic(()->left, vx, vy);
    }

    public Command shootSpeedup(){
        return shooter.reachGoal(70).withInterruptBehavior(InterruptionBehavior.kCancelSelf);
    }

    double lastLaunchLeft = 0;
    double lastLaunchRight = 0;
    public void commandPeriodic(){
        // dynamicScoringData = scoreMath.dynamicScore(GameData.getHubPose3d(), interpolate && !Robot.isSimulation()
        //     ,hoodRotationsToShootRotations(Units.radiansToRotations(HoodConstants.MIN_HOOD_ANGLE_RAD)), hoodRotationsToShootRotations(Units.radiansToRotations(HoodConstants.MAX_HOOD_ANGLE_RAD)), rpsTomps(100));

        // dynamicPassLeft = scoreMath.dynamicScore(GameData.getPassPose3d(true), interpolate && !Robot.isSimulation()
        //     ,Units.radiansToRotations(HoodConstants.MIN_HOOD_ANGLE_RAD), hoodRotationsToShootRotations(Units.radiansToRotations(HoodConstants.MAX_HOOD_ANGLE_RAD)), rpsTomps(100));
        // dynamicPassRight = scoreMath.dynamicScore(GameData.getPassPose3d(false), interpolate && !Robot.isSimulation()
        //     ,Units.radiansToRotations(HoodConstants.MIN_HOOD_ANGLE_RAD), hoodRotationsToShootRotations(Units.radiansToRotations(HoodConstants.MAX_HOOD_ANGLE_RAD)), rpsTomps(100));
        dynamicScoringData = scoreMath.staticScore(GameData.getHubPose3d(), interpolate, shootAngle, rpsTomps(100));
        dynamicPassLeft = scoreMath.staticScore(GameData.getPassPose3d(true), interpolate, shootAngle, rpsTomps(100));
        dynamicPassRight = scoreMath.staticScore(GameData.getPassPose3d(false), interpolate, shootAngle, rpsTomps(100));

        // System.out.println(PoseEX.correctedRotation(dynamicScoringData[2]-swerveDrive.getPose().getRotation().getRotations()));
        if (!interpolate){
            dynamicScoringData = new double[]{shootAngle, mpsToRps(dynamicScoringData[1])*1.2, dynamicScoringData[2]};
            dynamicPassLeft = new double[]{shootAngle, mpsToRps(dynamicPassLeft[1])*1.2, dynamicPassLeft[2]};
            dynamicPassRight = new double[]{shootAngle, mpsToRps(dynamicPassRight[1])*1.2, dynamicPassRight[2]};
        }
        if (shooter.getVelocity() > 1 && indexer.getVelocity() > 2 && Utils.getCurrentTimeSeconds() > lastLaunchLeft + .2){
            lastLaunchLeft = Utils.getCurrentTimeSeconds();
            animations.addFlyingObject(swerveDrive.getPose(), fromSwerveBase.getTranslation(), new Rotation3d(0,Units.rotationsToRadians(.25-shootAngle),Units.rotationsToRadians(0)), swerveDrive.getSpeeds(), rpsTomps(shooter.getVelocity()/1.07));
        }
        // if (!Robot.isSimulation() && rightShooter.getVelocity() > 1 && rightIndexer.getVelocity() > 2 && Utils.getCurrentTimeSeconds() > lastLaunchRight + .2){
        //     lastLaunchRight = Utils.getCurrentTimeSeconds();
        //     animations.addFlyingObject(swerveDrive.getPose(), fromSwerveBase.getTranslation(), new Rotation3d(0,Units.rotationsToRadians(.25-hoodRotationsToShootRotations(hood.getPosition())),Units.rotationsToRadians(0)), swerveDrive.getSpeeds(), rpsTomps(rightShooter.getVelocity()));
        // }
        Pose2d turretPose = new Pose3d(swerveDrive.getPose()).transformBy(fromSwerveBase).toPose2d();
        aimCorrect.accept(readyToShootHub());
        distance.accept(PoseEX.getDistanceFromPoseMeters(turretPose, GameData.getHubPose2d()));
        entry.accept(ScoreMath.divisor);
    }
}