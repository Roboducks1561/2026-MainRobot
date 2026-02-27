package frc.robot.subsystems;

import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.commands.ArmSlow;
import frc.robot.subsystems.TurretMechanism.Hood;
import frc.robot.subsystems.TurretMechanism.Indexer;
import frc.robot.subsystems.TurretMechanism.Shooter;
import frc.robot.subsystems.intakeMechanism.Arm;
import frc.robot.subsystems.intakeMechanism.Intake;
import frc.robot.subsystems.intakeMechanism.Spindexer;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.util.SendableConsumer;

public class BaseMechanism {

    private final Notifier notifier;

    public final Arm arm;
    public final Intake intake;
    public final Indexer leftIndexer;
    public final Indexer rightIndexer;
    public final Spindexer spindexer;
    public final Hood hood;
    public final Shooter leftShooter;
    public final Shooter rightShooter;
    public final SwerveDrive swerveDrive;

    protected final double intakeSpeed = 20;
    protected final double indexSpeed = 10;
    protected final double spinSpeed = 30;

    protected final double armIntakePosition = .34;
    protected final double shooterDefaultSpeed = 5;

    protected int hopperState = 0;
    protected int lastHopperState = 0;
    protected boolean invertedIntake = false;

    protected final Transform3d fromSwerveBase = new Transform3d(-.24,0,.326, new Rotation3d());

    public final Set<Subsystem> smartShootRequirements;
    public final Set<Subsystem> shooterRequirements;
    public final Set<Subsystem> intakeRequirements;

    private final NetworkTable robot = NetworkTableInstance.getDefault().getTable("Robot");
    protected final NetworkTable readyToShootRequirements = robot.getSubTable("ShootRequirements");
    private final BooleanPublisher hoodCorrect = readyToShootRequirements
        .getBooleanTopic("HoodCorrect").publish();
    private final BooleanPublisher leftShooterCorrect = readyToShootRequirements
        .getBooleanTopic("leftShooterCorrect").publish();
    private final BooleanPublisher rightShooterCorrect = readyToShootRequirements
        .getBooleanTopic("rightShooterCorrect").publish();


    public BaseMechanism(Arm arm, Intake intake, Indexer leftIndexer, Indexer rightIndexer,Shooter leftShooter,Shooter rightShooter, Spindexer spindexer, Hood hood, SwerveDrive swerveDrive){

        this.arm = arm;
        this.intake = intake;
        this.leftIndexer = leftIndexer;
        this.rightIndexer = rightIndexer;
        this.spindexer = spindexer;
        this.hood = hood;
        this.leftShooter = leftShooter;
        this.rightShooter = rightShooter;
        this.swerveDrive = swerveDrive;

        smartShootRequirements = Set.of(leftIndexer, rightIndexer, spindexer, hood, leftShooter, rightShooter, swerveDrive);
        shooterRequirements = Set.of(leftIndexer, rightIndexer, spindexer, hood, leftShooter, rightShooter);
        intakeRequirements = Set.of(intake, arm);


        arm.setDefaultCommand((
            Commands.either(arm.reachGoal(armIntakePosition)
            ,Commands.either(arm.reachGoal(()->(int)(Utils.getCurrentTimeSeconds() * 5)%2 == 1 ? .2: .1)
            ,new ArmSlow(arm, 1, 0).until(()->arm.getPosition()-.05 < 0).andThen(arm.setVoltage(-1.3))
            , ()->hopperState == 1)
            , ()->hopperState == 2)
        .until(()->{
            boolean b = lastHopperState != hopperState;
            if (b){
                lastHopperState = hopperState;
            }
            return b;
        })).repeatedly());
        intake.setDefaultCommand(intake.reachGoal(()->!invertedIntake ? (int)(Utils.getCurrentTimeSeconds() * 5)%4 == 0 ? -intakeSpeed/24: intakeSpeed/24 : -intakeSpeed/2));
        leftIndexer.setDefaultCommand(leftIndexer.reachGoal(()->leftIndexer.hasPiece() ? 0 : indexSpeed/4));
        rightIndexer.setDefaultCommand(rightIndexer.reachGoal(()->rightIndexer.hasPiece() ? 0 : indexSpeed/4));
        spindexer.setDefaultCommand(spindexer.reachGoal(()->leftIndexer.hasPiece() && rightIndexer.hasPiece() ? 0 : spinSpeed/30));
        
        hood.setDefaultCommand(hood.reachGoal(0));
        leftShooter.setDefaultCommand(leftShooter.reachGoal(0));
        rightShooter.setDefaultCommand(rightShooter.reachGoal(0));

        notifier = new Notifier(this :: periodic);
        notifier.setName("BaseMechanism Periodic");
        notifier.startPeriodic(.02);
        Runtime.getRuntime().addShutdownHook(new Thread(notifier::close));

        zeroSetter();
        // defaultSetter();
    }

    public boolean readyToShootLeft(){
        return leftShooter.withinBounds()
        && hood.withinBounds()
        && leftShooter.getTargetVelocity() != 0;
    }
    
    public boolean readyToShootRight(){
        return rightShooter.withinBounds()
        && hood.withinBounds()
        && rightShooter.getTargetVelocity() != 0;
    }

    public Command intake(){
        return Commands.parallel(arm.reachGoal(armIntakePosition), intakeRollers());
    }

    public Command intakeRollers(){
        return intake.reachGoal(()->!invertedIntake ? (int)(Utils.getCurrentTimeSeconds() * 3)%6 == 0 ? intakeSpeed: intakeSpeed : -intakeSpeed/2);
    }

    public Command setIntakeNegative(){
        return Commands.idle().beforeStarting(()->invertedIntake = true).finallyDo(()->invertedIntake = false);
    }

    public Command hopperOut(){
        return arm.reachGoal(armIntakePosition).alongWith(intake.reachGoal(0));
    }

    public Command overDepot(){
        return Commands.parallel(arm.reachGoal(armIntakePosition - .1), intakeRollers());
    }

    public Command spindex(){
        return (spindexer.reachGoal(spinSpeed).withTimeout(1.0).andThen(spindexer.reachGoal(-spinSpeed).withTimeout(.1))).repeatedly();
    }

    /**
     * 0 being up, 1 being shaking, and 2 down
     * @param state
     * @return
     */
    public void setHopperState(int state){
        hopperState = state;
    }

    /*
     * 
     * @param pivotRotation
     * @param velocityRps
     * @param turretRotation
     * @param ready: Only an additional requirement; does general shooter up to speed automatically
     * @return 
     */

    public Command shootLeft(double pivotRotation, double velocityRps, double turretRotation, BooleanSupplier ready){
        return Commands.parallel(leftShooter.reachGoal(velocityRps), hood.reachGoal(pivotRotation), spindexer.reachGoal(spinSpeed))
            .until(()->ready.getAsBoolean() && readyToShootLeft()).andThen(leftIndexer.reachGoal(indexSpeed));
    }

    public Command shootRight(double pivotRotation, double velocityRps, double turretRotation, BooleanSupplier ready){
        return Commands.parallel(rightShooter.reachGoal(velocityRps), hood.reachGoal(pivotRotation), spindexer.reachGoal(spinSpeed))
            .until(()->ready.getAsBoolean() && readyToShootRight()).andThen(rightIndexer.reachGoal(indexSpeed));
    }

    /**
     * exempt hood and spindexer
     * @param hood
     * @param shooter
     * @param pivotRotation
     * @param velocityRps
     * @param turretRotation
     * @param ready the whole boolean of ready, not just additions
     * @return
     */
    public Command shootContinuous(Shooter shooter, Indexer indexer, DoubleSupplier velocityRps, BooleanSupplier ready){
        return Commands.parallel(shooter.reachGoal(velocityRps)
            ,indexer.reachGoal(()-> ready.getAsBoolean() ? indexSpeed : 0));
    }

    public Command shootContinuousLeft(DoubleSupplier pivotRotation, DoubleSupplier velocityRps, DoubleSupplier turretRotation, BooleanSupplier ready){
        return Commands.parallel(hood.reachGoal(pivotRotation)
            ,shootContinuous(leftShooter, leftIndexer, velocityRps, ()-> (ready.getAsBoolean() && readyToShootLeft()) || !leftIndexer.hasPiece())
            ,spindex());
    }

    public Command shootContinuousRight(DoubleSupplier pivotRotation, DoubleSupplier velocityRps, DoubleSupplier turretRotation, BooleanSupplier ready){
        return Commands.parallel(hood.reachGoal(pivotRotation)
            ,shootContinuous(rightShooter, rightIndexer, velocityRps, ()-> ready.getAsBoolean() && readyToShootRight() || !rightIndexer.hasPiece())
            ,spindex());
    }

    public Command shootBothContinuous(DoubleSupplier pivotRotation, DoubleSupplier velocityRps, DoubleSupplier turretRotation, BooleanSupplier ready){
        return Commands.parallel(hood.reachGoal(pivotRotation)
            ,shootContinuous(leftShooter, leftIndexer, velocityRps, ()-> ready.getAsBoolean() && readyToShootLeft() || !leftIndexer.hasPiece())
            ,shootContinuous(rightShooter, rightIndexer, ()-> velocityRps.getAsDouble() * 1.03, ()-> ready.getAsBoolean() && readyToShootRight() || !rightIndexer.hasPiece())
            ,spindex());
    }

    /**
     * For testing only
     */
    public void defaultSetter(){
        double[] values = new double[]{0,0,0,0,0,0,0,0};
        arm.setDefaultCommand(arm.reachGoal(()->values[0]));
        intake.setDefaultCommand(intake.reachGoal(()->values[1]));
        leftIndexer.setDefaultCommand(leftIndexer.reachGoal(()->values[2]));
        rightIndexer.setDefaultCommand(rightIndexer.reachGoal(()->values[3]));
        spindexer.setDefaultCommand(spindexer.reachGoal(()->values[4]));
        hood.setDefaultCommand(hood.reachGoal(()->values[5]));
        leftShooter.setDefaultCommand(leftShooter.reachGoal(()->values[6]));
        rightShooter.setDefaultCommand(rightShooter.reachGoal(()->values[7]));
        DoubleEntry[] defaultSetters = SendableConsumer.createSendableChooser("Defaults",new String[]{"arm","intake","leftIndexer","rightIndexer","spindexer","hood","leftShooter","rightShooter"}, new double[]{0,0,0,0,0,0,0,0});
        SendableConsumer.checker(defaultSetters, new DoubleConsumer[]{
            (i)->values[0] = i
            ,(i)->values[1] = i
            ,(i)->values[2] = i
            ,(i)->values[3] = i
            ,(i)->values[4] = i
            ,(i)->values[5] = i
            ,(i)->values[6] = i
            ,(i)->values[7] = i
        });
    }

    public void zeroSetter(){
        BooleanEntry zeroSetter = SendableConsumer.createSendableChooser("zeroSetter",false);
        SendableConsumer.checker(zeroSetter, (b)->{
            zeroSetter.set(false);
            arm.setZero();
            hood.setZero();
        });
    }

    public void periodic(){
        hoodCorrect.accept(hood.withinBounds());
        leftShooterCorrect.accept(leftShooter.withinBounds());
        rightShooterCorrect.accept(rightShooter.withinBounds());
    }
}
