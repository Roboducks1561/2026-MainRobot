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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
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
    public final Indexer indexer;
    public final Spindexer spindexer;
    public final Hood hood;
    public final Shooter shooter;
    public final SwerveDrive swerveDrive;

    protected final double intakeSpeed = 30;
    protected final double indexSpeed = 20;
    protected final double spinSpeed = 10;

    protected final double armIntakePosition = .34;
    protected final double shooterDefaultSpeed = 5;

    protected DoubleSupplier hopperPos = ()->0;
    protected double lastHopperPos = 0;
    protected boolean invertedIntake = false;
    protected boolean invertedSpindexer = false;

    protected final Transform3d fromSwerveBase = new Transform3d(-.24,0,.326, new Rotation3d());

    public final Set<Subsystem> smartShootRequirements;
    public final Set<Subsystem> shooterRequirements;
    public final Set<Subsystem> intakeRequirements;

    private final NetworkTable robot = NetworkTableInstance.getDefault().getTable("Robot");
    protected final NetworkTable readyToShootRequirements = robot.getSubTable("ShootRequirements");
    private final BooleanPublisher hoodCorrect = readyToShootRequirements
        .getBooleanTopic("HoodCorrect").publish();
    private final BooleanPublisher shooterCorrect = readyToShootRequirements
        .getBooleanTopic("shooterCorrect").publish();



    public BaseMechanism(Arm arm, Intake intake, Indexer indexer,Shooter shooter, Spindexer spindexer, Hood hood, SwerveDrive swerveDrive){
        this.arm = arm;
        this.intake = intake;
        this.indexer = indexer;
        this.spindexer = spindexer;
        this.hood = hood;
        this.shooter = shooter;
        this.swerveDrive = swerveDrive;

        smartShootRequirements = Set.of(indexer, spindexer, hood, shooter, swerveDrive);
        shooterRequirements = Set.of(indexer, spindexer, hood, shooter);
        intakeRequirements = Set.of(intake, arm);
        // arm.setDefaultCommand(arm.reachGoal(()->DriverStation.isAutonomous() ? 0 : hopperPos.getAsDouble() * armIntakePosition));
        arm.setDefaultCommand(Commands.either(arm.reachGoal(0).until(()->arm.getPosition() - .05 < 0).andThen(arm.setVoltage(-1.3)), arm.reachGoal(()->hopperPos.getAsDouble() * armIntakePosition), ()->hopperPos.getAsDouble() == 0)
        .until(()->{
            boolean b = lastHopperPos != hopperPos.getAsDouble();
            if (b){
                lastHopperPos = hopperPos.getAsDouble();
            }
            return b;
        }));
        intake.setDefaultCommand(intake.reachGoal(()->!invertedIntake ? intakeSpeed/24 : -intakeSpeed/2));
        indexer.setDefaultCommand(indexer.reachGoal(0));
        spindexer.setDefaultCommand(spindexer.reachGoal(()->!invertedSpindexer ? 0 : -spinSpeed));
        
        hood.setDefaultCommand(hood.reachGoal(0));
        shooter.setDefaultCommand(shooter.reachGoal(0));

        notifier = new Notifier(this :: periodic);
        notifier.setName("BaseMechanism Periodic");
        notifier.startPeriodic(.02);
        Runtime.getRuntime().addShutdownHook(new Thread(notifier::close));

        zeroSetter();
        defaultSetter();
    }

    public boolean readyToShoot(){
        return shooter.withinBounds()
        && hood.withinBounds()
        && shooter.getTargetVelocity() != 0;
    }

    public Command stopShooting(){
        return Commands.parallel(shooter.reachGoalOnce(0),
            hood.reachGoalOnce(0), 
            spindexer.reachGoalOnce(0), 
            indexer.reachGoalOnce(0));
    }

    public Command stopIntake(){
        return pulseIntake().withTimeout(.5).andThen(
            Commands.parallel(pulseIntake()
            ,arm.reachGoal(0).until(()->arm.getPosition()-.1 < 0).andThen(arm.setVoltage(-1.3))));
    }

    public Command intake(){
        return Commands.parallel(arm.reachGoal(armIntakePosition), intakeRollers());
    }

    public Command intakeRollers(){
        // return (intake.reachGoal(intakeSpeed).until(()->intake.getCurrent() > 60).andThen(intake.reachGoal(-intakeSpeed).withTimeout(.1))).repeatedly();
        return intake.reachGoal(()->!invertedIntake ? (int)(Utils.getCurrentTimeSeconds() * 3)%6 == 0 ? intakeSpeed: intakeSpeed : -intakeSpeed/2);
    }

    public Command setIntakeNegative(){
        return Commands.idle().beforeStarting(()->invertedIntake = true).finallyDo(()->invertedIntake = false);
    }

    public Command hopperShake(){
        return arm.reachGoal(()->((int)(Utils.getCurrentTimeSeconds()*3))%2 == 0 ? .25 : .05).withInterruptBehavior(InterruptionBehavior.kCancelSelf);
    }

    public Command pulseIntake(){
        return intake.reachGoal(()->((int)(Utils.getCurrentTimeSeconds()*3))%3 == 0 ? -intakeSpeed : intakeSpeed);
    }

    public Command hopperOut(){
        return arm.reachGoal(armIntakePosition).alongWith(intake.reachGoal(0));
    }

    public Command overDepot(){
        return Commands.parallel(arm.stop(), intakeRollers());
    }

    public Command spindex(){
        return spindexer.reachGoal(()->!invertedSpindexer ? spinSpeed : -spinSpeed);//(spindexer.reachGoal(spinSpeed).withTimeout(1.0).andThen(spindexer.reachGoal(-spinSpeed).withTimeout(.2))).repeatedly();
    }

    public Command setSpindexNegative(){
        return Commands.idle().beforeStarting(()->invertedSpindexer = true).finallyDo(()->invertedSpindexer = false);
    }

    /**
     * 0 being up, 1 being shaking, and 2 down
     * @param state
     * @return
     */
    public void setHopperPos(DoubleSupplier state){
        hopperPos = state;
    }

    public Command shootContinuous(DoubleSupplier pivotRotation, DoubleSupplier velocityRps, DoubleSupplier turretRotation, BooleanSupplier ready){
        return Commands.parallel(hood.reachGoal(pivotRotation)
            ,shooter.reachGoal(velocityRps)
            ,indexer.reachGoal(()-> ready.getAsBoolean() && readyToShoot() ? indexSpeed : 0)
            ,spindex());
    }

    /**
     * For testing only
     */
    public void defaultSetter(){
        double[] values = new double[]{0,0,0,0,0,0,0,0};
        arm.setDefaultCommand(arm.reachGoal(()->values[0]));
        intake.setDefaultCommand(intake.reachGoal(()->values[1]));
        indexer.setDefaultCommand(indexer.reachGoal(()->values[2]));
        spindexer.setDefaultCommand(spindexer.reachGoal(()->values[3]));
        hood.setDefaultCommand(hood.reachGoal(()->values[4]));
        shooter.setDefaultCommand(shooter.reachGoal(()->values[5]));
        DoubleEntry[] defaultSetters = SendableConsumer.createSendableChooser("Defaults",new String[]{"arm","intake","indexer","spindexer","hood","shooter"}, new double[]{0,0,0,0,0,0});
        SendableConsumer.checker(defaultSetters, new DoubleConsumer[]{
            (i)->values[0] = i
            ,(i)->values[1] = i
            ,(i)->values[2] = i
            ,(i)->values[3] = i
            ,(i)->values[4] = i
            ,(i)->values[5] = i
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
        shooterCorrect.accept(shooter.withinBounds());
    }
}
