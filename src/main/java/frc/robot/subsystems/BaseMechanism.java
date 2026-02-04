package frc.robot.subsystems;

import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Robot;
import frc.robot.subsystems.TurretMechanism.Hood;
import frc.robot.subsystems.TurretMechanism.Shooter;
import frc.robot.subsystems.TurretMechanism.Turret;
import frc.robot.subsystems.climbMechanism.ClimbElevator;
import frc.robot.subsystems.intakeMechanism.Arm;
import frc.robot.subsystems.intakeMechanism.Indexer;
import frc.robot.subsystems.intakeMechanism.Intake;
import frc.robot.subsystems.intakeMechanism.Spindexer;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.util.mapleSim.Animations;
import frc.robot.util.mapleSim.Bootleg2026;

public class BaseMechanism {

    private final Notifier notifier;

    public final Arm arm;
    public final Intake intake;
    public final Indexer indexer;
    public final Spindexer spindexer;
    public final Hood hood;
    public final Shooter shooter;
    public final SwerveDrive swerveDrive;
    public final ClimbElevator climbElevator;

    protected final double intakeSpeed = 5;
    protected final double indexSpeed = 5;
    protected final double spinSpeed = 5;

    protected final double armIntakePosition = .25;
    protected final double shooterDefaultSpeed = 5;

    protected final double climberUpPosition = 1;

    protected final Transform3d fromSwerveBase = new Transform3d(-.24,0,.326, new Rotation3d());

    public final Set<Subsystem> smartShootRequirements;
    public final Set<Subsystem> shooterRequirements;
    public final Set<Subsystem> intakeRequirements;
    public final Set<Subsystem> climbRequirements;

    public BaseMechanism(Arm arm, Intake intake, Indexer indexer, Spindexer spindexer, Hood hood, Shooter shooter, ClimbElevator climbElevator, SwerveDrive swerveDrive){

        this.arm = arm;
        this.intake = intake;
        this.indexer = indexer;
        this.spindexer = spindexer;
        this.hood = hood;
        this.shooter = shooter;
        this.climbElevator = climbElevator;
        this.swerveDrive = swerveDrive;

        smartShootRequirements = Set.of(indexer, spindexer, hood, shooter, swerveDrive);
        shooterRequirements = Set.of(indexer, spindexer, hood, shooter);
        intakeRequirements = Set.of(intake, arm);
        climbRequirements = Set.of(arm, climbElevator);

        arm.setDefaultCommand(arm.reachGoal(0));
        intake.setDefaultCommand(intake.reachGoal(0));
        indexer.setDefaultCommand(indexer.reachGoal(0));
        spindexer.setDefaultCommand(spindexer.reachGoal(0));
        
        hood.setDefaultCommand(hood.reachGoal(0));
        shooter.setDefaultCommand(shooter.reachGoal(0));
        climbElevator.reachGoal(0);

        notifier = new Notifier(this :: periodic);
        notifier.setName("BaseMechanism Periodic");
        notifier.startPeriodic(.02);
        Runtime.getRuntime().addShutdownHook(new Thread(notifier::close));
    }

    public Command climbUp(){
        return climbElevator.reachGoal(climberUpPosition);
    }

    public Command climbDown(){
        return climbElevator.reachGoal(0);
    }

    public boolean readyToShoot(){
        return shooter.withinBounds()
        && hood.withinBounds()
        && shooter.getTargetVelocity() != 0;
    }

    public Command intake(){
        return Commands.parallel(arm.reachGoal(armIntakePosition), intake.reachGoal(intakeSpeed));
    }

    public Command shoot(double pivotRotation, double velocityRps, double turretRotation, BooleanSupplier ready){
        return Commands.parallel(shooter.reachGoal(velocityRps), hood.reachGoal(pivotRotation), spindexer.reachGoal(spinSpeed))
            .until(ready).andThen(indexer.reachGoal(indexSpeed));
    }

    public Command shootContinuous(DoubleSupplier pivotRotation, DoubleSupplier velocityRps, DoubleSupplier turretRotation, BooleanSupplier ready){
        return Commands.parallel(hood.reachGoal(pivotRotation), shooter.reachGoal(velocityRps)
            ,indexer.reachGoal(()-> ready.getAsBoolean() ? indexSpeed : 0), spindexer.reachGoal(spinSpeed));
    }

    public void periodic(){
        
    }
}
