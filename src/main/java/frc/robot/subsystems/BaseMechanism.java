package frc.robot.subsystems;

import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Robot;
import frc.robot.subsystems.TurretMechanism.Hood;
import frc.robot.subsystems.TurretMechanism.Indexer;
import frc.robot.subsystems.TurretMechanism.Shooter;
import frc.robot.subsystems.TurretMechanism.Turret;
import frc.robot.subsystems.intakeMechanism.Arm;
import frc.robot.subsystems.intakeMechanism.Intake;
import frc.robot.subsystems.intakeMechanism.Spindexer;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.util.SendableConsumer;
import frc.robot.util.mapleSim.Animations;
import frc.robot.util.mapleSim.Bootleg2026;

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

    protected final double intakeSpeed = 5;
    protected final double indexSpeed = 5;
    protected final double spinSpeed = 5;

    protected final double armIntakePosition = .25;
    protected final double shooterDefaultSpeed = 5;

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

        arm.setDefaultCommand(arm.reachGoal(0));
        intake.setDefaultCommand(intake.reachGoal(0));
        leftIndexer.setDefaultCommand(leftIndexer.reachGoal(0));
        rightIndexer.setDefaultCommand(rightIndexer.reachGoal(0));
        spindexer.setDefaultCommand(spindexer.reachGoal(0));
        
        hood.setDefaultCommand(hood.reachGoal(0));
        leftShooter.setDefaultCommand(leftShooter.reachGoal(0));
        rightShooter.setDefaultCommand(rightShooter.reachGoal(0));

        notifier = new Notifier(this :: periodic);
        notifier.setName("BaseMechanism Periodic");
        notifier.startPeriodic(.02);
        Runtime.getRuntime().addShutdownHook(new Thread(notifier::close));

        zeroSetter();
        defaultSetter();
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
        return Commands.parallel(arm.reachGoal(armIntakePosition), intake.reachGoal(intakeSpeed));
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
            .until(()->ready.getAsBoolean() && readyToShootRight()).andThen(leftIndexer.reachGoal(indexSpeed));
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
    public Command shootContinuous(Shooter shooter, Indexer indxer, DoubleSupplier velocityRps, BooleanSupplier ready){
        return Commands.parallel(shooter.reachGoal(velocityRps)
            ,indxer.reachGoal(()-> ready.getAsBoolean() ? indexSpeed : 0));
    }

    public Command shootContinuousLeft(DoubleSupplier pivotRotation, DoubleSupplier velocityRps, DoubleSupplier turretRotation, BooleanSupplier ready){
        return Commands.parallel(hood.reachGoal(pivotRotation)
            ,shootContinuous(leftShooter, leftIndexer, velocityRps, ()-> ready.getAsBoolean() && readyToShootLeft())
            ,spindexer.reachGoal(spinSpeed));
    }

    public Command shootContinuousRight(DoubleSupplier pivotRotation, DoubleSupplier velocityRps, DoubleSupplier turretRotation, BooleanSupplier ready){
        return Commands.parallel(hood.reachGoal(pivotRotation)
            ,shootContinuous(rightShooter, rightIndexer, velocityRps, ()-> ready.getAsBoolean() && readyToShootRight())
            ,spindexer.reachGoal(spinSpeed));
    }

    public Command shootBothContinuous(DoubleSupplier pivotRotation, DoubleSupplier velocityRps, DoubleSupplier turretRotation, BooleanSupplier ready){
        return Commands.parallel(hood.reachGoal(pivotRotation)
            ,shootContinuous(leftShooter, leftIndexer, velocityRps, ()-> ready.getAsBoolean() && readyToShootLeft())
            ,shootContinuous(rightShooter, rightIndexer, velocityRps, ()-> ready.getAsBoolean() && readyToShootRight())
            ,spindexer.reachGoal(spinSpeed));
    }

    /**
     * For testing only
     */
    public void defaultSetter(){
        double[] defaults = new double[]{0,0,0,0,0,0,0,0};
        arm.setDefaultCommand(arm.reachGoal(()->defaults[0]));
        intake.setDefaultCommand(intake.reachGoal(()->defaults[1]));
        leftIndexer.setDefaultCommand(leftIndexer.reachGoal(()->defaults[2]));
        rightIndexer.setDefaultCommand(rightIndexer.reachGoal(()->defaults[3]));
        spindexer.setDefaultCommand(spindexer.reachGoal(()->defaults[4]));
        hood.setDefaultCommand(hood.reachGoal(()->defaults[5]));
        leftShooter.setDefaultCommand(leftShooter.reachGoal(()->defaults[6]));
        rightShooter.setDefaultCommand(rightShooter.reachGoal(()->defaults[7]));
        SendableConsumer.createSendableChooser("arm", (i)->{
            defaults[0] = i;
        },0);
        SendableConsumer.createSendableChooser("intake", (i)->{
            defaults[1] = i;
        },0);
        SendableConsumer.createSendableChooser("leftIndexer", (i)->{
            defaults[2] = i;
        },0);
        SendableConsumer.createSendableChooser("rightIndexer", (i)->{
            defaults[3] = i;
        },0);
        SendableConsumer.createSendableChooser("spindexer", (i)->{
            defaults[4] = i;
        },0);
        SendableConsumer.createSendableChooser("hood", (i)->{
            defaults[5] = i;
        },0);
        SendableConsumer.createSendableChooser("leftShooter", (i)->{
            defaults[6] = i;
        },0);
        SendableConsumer.createSendableChooser("rightShooter", (i)->{
            defaults[7] = i;
        },0);
    }

    public void zeroSetter(){
        SendableChooser<Boolean> zeroSetter = new SendableChooser<>();

        zeroSetter.addOption("Set", true);
        zeroSetter.setDefaultOption("Default", false);

        SmartDashboard.putData("SetZeros", zeroSetter);

        zeroSetter.onChange((value)->{
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
