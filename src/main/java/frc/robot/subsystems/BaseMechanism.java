package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.robot.Robot;
import frc.robot.constants.GameData;
import frc.robot.subsystems.TurretMechanism.Hood;
import frc.robot.subsystems.TurretMechanism.Shooter;
import frc.robot.subsystems.TurretMechanism.Turret;
import frc.robot.subsystems.intakeMechanism.Arm;
import frc.robot.subsystems.intakeMechanism.Indexer;
import frc.robot.subsystems.intakeMechanism.Intake;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.util.PoseEX;
import frc.robot.util.mapleSim.Bootleg2022;

public class BaseMechanism {

    //This is tuned later
    protected final double shootConversion = 1;//1/0.238;

    protected final double shootAngleConversion = 1;
    protected final double shootAngleOffset = -0.06446;

    private final Notifier notifier;

    public final double MAX_TURRET_ERROR = .01;
    public final double MAX_HOOD_ERROR = .01;
    public final double MAX_SHOOTER_ERROR = .1;
    public final double MAX_ARM_ERROR = .03;

    public final double shootTime = 1;
    public final double intakeTime = .3;

    public final Arm arm;
    public final Intake intake;

    public final Indexer indexer;

    public final Turret turret;
    public final Hood hood;
    public final Shooter shooter;

    public final SwerveDrive swerveDrive;

    public final Set<Subsystem> subsystems;

    protected double last = 0;
    protected double timeSinceLast = 0;

    protected double turretTotalRotation = 0;


    public static enum MainStates{
        Rest(-.24,0,0.25),
        Intake(-.07,0,0.25),
        ShootForward(-.24,0,.25),
        ShootRight(-.24,-.25,.25),
        ShootLeft(-.24,.25,.25);

        public double armRotation;
        public double turretRotation;
        public double hoodRotation;

        public static double intakeSpeed = 60;
        public static double intakeFeedSpeed = 60;

        public static double indexerFeedSpeed = 100;
        public static double indexerWaitSpeed = 20;

        MainStates(double armRotation, double turretRotation, double hoodRotation){
            this.armRotation = armRotation;
            this.turretRotation = turretRotation;
            this.hoodRotation = hoodRotation;
        }
    }




    public BaseMechanism(Arm arm, Intake intake, Indexer indexer, Turret turret, Hood hood, Shooter shooter, SwerveDrive swerveDrive){
        this.arm = arm;
        this.intake = intake;
        this.indexer = indexer;
        this.turret = turret;
        this.hood = hood;
        this.shooter = shooter;
        this.swerveDrive = swerveDrive;

        this.subsystems = Set.of(arm, intake, indexer, turret, hood, shooter, swerveDrive);

        arm.setDefaultCommand(arm.reachGoal(MainStates.Rest.armRotation));

        intake.setDefaultCommand(intake.reachGoal(()->((intake.hasPiece() && !indexer.hasPiece()) && timeSinceLast < 3) ? MainStates.intakeFeedSpeed : 0));
        indexer.setDefaultCommand(indexer.reachGoal(()->((intake.hasPiece() && !indexer.hasPiece()) || (timeSinceLast < 3 && !indexer.hasPiece())) ? MainStates.indexerWaitSpeed : 0));

        turret.setDefaultCommand(turret.reachGoal(MainStates.Rest.turretRotation));
        hood.setDefaultCommand(hood.reachGoal(MainStates.Rest.hoodRotation));
        shooter.setDefaultCommand(shooter.reachGoal(0));

        if (Robot.isSimulation()){
            Bootleg2022.addShooterSimulation(
                ()->turret.fromSwerveBase.plus(new Transform3d(0,0,0,new Rotation3d(0,Units.rotationsToRadians(hood.invertInterpolation(hood.getPosition())),Units.rotationsToRadians(turret.getPosition()))))
                    ,()->shooter.getVelocity() * 1.07, "Algae", "Intake");
            Bootleg2022.addShootRequirements("Intake", ()->shooter.getVelocity() > 5 && indexer.getVelocity() > 5);

            Bootleg2022.addIntakeSimulation("Intake","Algae",1,1,new Translation2d(-.3,0));
            Bootleg2022.addIntakeRequirements("Intake", ()->Math.abs(arm.getPosition() - MainStates.Intake.armRotation) < MAX_ARM_ERROR);

            int[] lastI = new int[]{1};
            Bootleg2022.hasPiece("Intake",(i)->{
                intake.getMotorStrainIO().setValue(lastI[0] < i);
                lastI[0] = i;
                intake.getDigitalInputIO().setValue(i == 2);
                indexer.getDigitalInputIO().setValue(i == 1 || i == 2);
            });
        }
        notifier = new Notifier(this :: periodic);
        notifier.setName("Scoring Periodic");
        notifier.startPeriodic(.02);
        Runtime.getRuntime().addShutdownHook(new Thread(notifier::close));
    }

    public Command intake(){
        return intake.reachGoal(MainStates.intakeSpeed)
        .alongWith(arm.reachGoal(MainStates.Intake.armRotation))
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    }

    public boolean readyToSmartShoot(){
        return Math.abs(PoseEX.correctedRotation(turretTotalRotation - PoseEX.getPoseAngle(swerveDrive.getPose(), GameData.scorePose2d).getRotations())) < MAX_TURRET_ERROR
            && Math.abs(hood.getPosition() - hood.getTarget()) < MAX_HOOD_ERROR
            && Math.abs(shooter.getVelocity() - shooter.getTargetVelocity()) < MAX_SHOOTER_ERROR
            && Math.abs(swerveDrive.getSpeeds().omegaRadiansPerSecond) < 1;
            // && avgAccelLast10Iterations < .05;
    }


    public boolean readyToShoot(){
        return Math.abs(turret.getPosition() - turret.getTarget()) < MAX_TURRET_ERROR
            && Math.abs(hood.getPosition() - hood.getTarget()) < MAX_HOOD_ERROR
            && Math.abs(shooter.getVelocity() - shooter.getTargetVelocity()) < MAX_SHOOTER_ERROR
            && Math.abs(swerveDrive.getSpeeds().omegaRadiansPerSecond) < 1;
            // && avgAccelLast10Iterations < .05;
    }

    public Command shoot(double turretRotation, double hoodRotation, double shootSpeed){
        return shoot(()->turretRotation, ()->hoodRotation, ()->shootSpeed);
    }

    public Command shoot(DoubleSupplier turretRotation, DoubleSupplier hoodRotation, DoubleSupplier shootSpeed){
        return Commands.race(shoot(shootSpeed, false), turret.reachGoal(turretRotation), hood.reachGoal(hoodRotation));
    }

    public Command shoot(double robotRotation, double turretRotation, double hoodRotation, double shootSpeed){
        return shoot(()->robotRotation, ()->turretRotation, ()->hoodRotation, ()->shootSpeed);
    }

    public Command shoot(DoubleSupplier robotRotation, DoubleSupplier turretRotation, DoubleSupplier hoodRotation, DoubleSupplier shootSpeed){
        return Commands.deadline(shoot(shootSpeed, true), turret.reachGoal(turretRotation), hood.reachGoal(hoodRotation), arm.reachGoal(MainStates.ShootForward.armRotation),
        swerveDrive.rotateTo(()->Rotation2d.fromRotations(robotRotation.getAsDouble()), Math.PI*3)
            .until(()->Math.abs(PoseEX.correctedRotation(turretTotalRotation - PoseEX.getPoseAngle(swerveDrive.getPose(), GameData.scorePose2d).getRotations())) < MAX_TURRET_ERROR)
            .andThen(swerveDrive.brake()));
    }

    protected Command shoot(double shootSpeed, boolean smartShoot){
        return shoot(shootSpeed, smartShoot);
    }

    /**
     * this assumes you are already aimed
     * @param turretRotation
     * @param hoodRotation
     * @param shootSpeed
     * @return
     */
    protected Command shoot(DoubleSupplier shootSpeed, boolean smartShoot){
        return Commands.either(Commands.race(
            shooter.reachGoal(()->Robot.isSimulation() ? shootSpeed.getAsDouble() : (0.383094*Math.pow(shootSpeed.getAsDouble(),2) + 3.38455*shootSpeed.getAsDouble() + 0.0834742)/1.2),
            Commands.waitUntil(()->smartShoot ? readyToSmartShoot() : readyToShoot()).andThen(
                Commands.either(
                    indexer.reachGoal(MainStates.indexerFeedSpeed).until(()->!indexer.hasPiece()).andThen(Commands.waitSeconds(shootTime))
                        .andThen(intake.reachGoal(MainStates.intakeFeedSpeed).alongWith(indexer.reachGoal(MainStates.indexerWaitSpeed)))
                        .until(()->indexer.hasPiece())
                        .andThen(Commands.waitUntil(()->smartShoot ? readyToSmartShoot() : readyToShoot()))
                    , Commands.none()
                    
                    , ()->intake.hasPiece()
                )
                .andThen(indexer.reachGoal(MainStates.indexerFeedSpeed).until(()->!indexer.hasPiece()).andThen(Commands.waitSeconds(shootTime)))
            )
        ),Commands.none(), ()->indexer.hasPiece());
    }

    public Command testShoot(double speed, double hoodRotation){
        return Commands.parallel(shooter.reachGoal(speed),hood.reachGoal(hoodRotation)).until(()->Math.abs(shooter.getVelocity() - shooter.getTargetVelocity()) < MAX_SHOOTER_ERROR).andThen(indexer.reachGoal(1000));
    }

    public Command reset(){
        return shooter.reachGoalOnce(0)
        .alongWith(indexer.reachGoalOnce(0))
        .alongWith(intake.reachGoalOnce(0))
        .alongWith(turret.reachGoalOnce(MainStates.Rest.turretRotation))
        .alongWith(hood.reachGoalOnce(MainStates.Rest.hoodRotation))
        .alongWith(arm.reachGoalOnce(MainStates.Rest.armRotation));
    }

    double avgAccelLast10Iterations = 0;

    ArrayList<Double> accelerations = new ArrayList<>(List.of(0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0));
    public void periodic(){
    //    System.out.println((Math.abs(PoseEX.correctedRotation(turretTotalRotation - PoseEX.getPoseAngle(swerveDrive.getPose(), GameData.scorePose2d).getRotations())) < MAX_TURRET_ERROR)
    //     +"    "+(Math.abs(hood.getPosition() - hood.getTarget()) < MAX_HOOD_ERROR)
    //     +"   "+(Math.abs(shooter.getVelocity() - shooter.getTargetVelocity()) < MAX_SHOOTER_ERROR)
    //     +"   "+(Math.abs(swerveDrive.getSpeeds().omegaRadiansPerSecond) < 1));
        // +"    "+(avgAccelLast10Iterations < .05));
        // System.out.println(arm.getPosition() +"   "+arm.getTarget());
        // System.out.println(turretTotalRotation +"   "+ PoseEX.getPoseAngle(swerveDrive.getPose(), GameData.scorePose2d).getRotations()+"   "+PoseEX.correctedRotation(turretTotalRotation - PoseEX.getPoseAngle(swerveDrive.getPose(), GameData.scorePose2d).getRotations()));
        accelerations.remove(0);
        accelerations.add(Math.sqrt(Math.pow(swerveDrive.getAcceleration().vxMetersPerSecond,2) + Math.pow(swerveDrive.getAcceleration().vyMetersPerSecond,2)));
        avgAccelLast10Iterations = 0;
        accelerations.forEach((i)->{
            avgAccelLast10Iterations +=i;
        });
        avgAccelLast10Iterations/=20;

        turretTotalRotation = swerveDrive.getPose().getRotation().getRotations() + turret.getPosition();
        if (intake.hasPiece()){
            last = Utils.getCurrentTimeSeconds();
        }
        // System.out.println(Utils.getSystemTimeSeconds() +"   "+ last);
        timeSinceLast = Utils.getSystemTimeSeconds() - last;
    }
}
