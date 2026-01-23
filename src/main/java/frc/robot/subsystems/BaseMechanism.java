package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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

public class BaseMechanism {

    private final Notifier notifier;

    protected final Arm arm;
    protected final Intake intake;
    protected final Indexer indexer;
    protected final Spindexer spindexer;
    protected final Turret turret;
    protected final Hood hood;
    protected final Shooter shooter;
    protected final SwerveDrive swerveDrive;
    protected final ClimbElevator climbElevator;
    protected final CommandXboxController xboxController;

    protected final Animations animations;

    protected final double intakeSpeed = 5;
    protected final double indexSpeed = 5;
    protected final double spinSpeed = 5;

    protected final double armIntakePosition = .25;
    protected final double shooterDefaultSpeed = 5;

    protected final double climberUpPosition = 1;

    public BaseMechanism(Arm arm, Intake intake, Indexer indexer, Spindexer spindexer, Turret turret, Hood hood, Shooter shooter, ClimbElevator climbElevator, SwerveDrive swerveDrive, CommandXboxController xboxController){

        this.arm = arm;
        this.intake = intake;
        this.indexer = indexer;
        this.spindexer = spindexer;
        this.turret = turret;
        this.hood = hood;
        this.shooter = shooter;
        this.climbElevator = climbElevator;
        this.swerveDrive = swerveDrive;
        this.xboxController = xboxController;

        animations = new Animations();

        arm.setDefaultCommand(arm.reachGoal(0));
        intake.setDefaultCommand(intake.reachGoal(0));
        indexer.setDefaultCommand(indexer.reachGoal(0));
        spindexer.setDefaultCommand(spindexer.reachGoal(0));
        
        hood.setDefaultCommand(hood.reachGoal(0));
        shooter.setDefaultCommand(shooter.reachGoal(0));
        climbElevator.reachGoal(0);

        if (Robot.isSimulation()){
            // Bootleg2026.addShooterSimulation(
            //     ()->turret.fromSwerveBase.plus(new Transform3d(0,0,0,new Rotation3d(0,Units.rotationsToRadians(hood.invertInterpolation(hood.getPosition())),Units.rotationsToRadians(turret.getPosition()))))
            //         ,()->shooter.getVelocity() * 1.07, "Algae", "Intake");
            // Bootleg2026.addShootRequirements("Intake", ()->shooter.getVelocity() > 5 && indexer.getVelocity() > 5);

            // Bootleg2026.addIntakeSimulation("Intake","Algae",1,1,new Translation2d(-.3,0));
            // Bootleg2026.addIntakeRequirements("Intake", ()->Math.abs(arm.getPosition() - MainStates.Intake.armRotation) < MAX_ARM_ERROR);

            // int[] lastI = new int[]{1};
            // Bootleg2026.hasPiece("Intake",(i)->{
            //     intake.getMotorStrainIO().setValue(lastI[0] < i);
            //     lastI[0] = i;
            //     intake.getDigitalInputIO().setValue(i == 2);
            //     indexer.getDigitalInputIO().setValue(i == 1 || i == 2);
            // });
        }

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
        && turret.withinBounds()
        && hood.withinBounds();
    }

    public Command intake(){
        return Commands.parallel(arm.reachGoal(armIntakePosition), intake.reachGoal(intakeSpeed));
    }

    public Command shoot(double pivotRotation, double velocityRps, double turretRotation){
        return Commands.parallel(shooter.reachGoal(velocityRps), hood.reachGoal(pivotRotation), turret.reachGoal(turretRotation), spindexer.reachGoal(spinSpeed))
            .until(()->readyToShoot()).andThen(indexer.reachGoal(indexSpeed));
    }

    public Command shootContinuous(DoubleSupplier pivotRotation, DoubleSupplier velocityRps, DoubleSupplier turretRotation){
        return Commands.parallel(hood.reachGoal(pivotRotation), shooter.reachGoal(velocityRps), turret.reachGoal(turretRotation)
            ,indexer.reachGoal(()-> readyToShoot() ? indexSpeed : 0), spindexer.reachGoal(spinSpeed));
    }

    double lastLaunch = 0;
    public void periodic(){
        if (readyToShoot() && indexer.getVelocity() > 2 && Utils.getCurrentTimeSeconds() > lastLaunch + .2 ){
            lastLaunch = Utils.getCurrentTimeSeconds();
            animations.addFlyingObject(swerveDrive.getPose(), turret.fromSwerveBase.getTranslation(), new Rotation3d(0,Units.rotationsToRadians(.25-hood.getPosition()),Units.rotationsToRadians(turret.getPosition())), swerveDrive.getSpeeds(), shooter.getVelocity());
        }
    }
}
