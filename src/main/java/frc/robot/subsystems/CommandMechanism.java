package frc.robot.subsystems;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Robot;
import frc.robot.constants.GameData;
import frc.robot.subsystems.TurretMechanism.Hood;
import frc.robot.subsystems.TurretMechanism.Shooter;
import frc.robot.subsystems.TurretMechanism.Turret;
import frc.robot.subsystems.climbMechanism.ClimbElevator;
import frc.robot.subsystems.intakeMechanism.Arm;
import frc.robot.subsystems.intakeMechanism.Indexer;
import frc.robot.subsystems.intakeMechanism.Intake;
import frc.robot.subsystems.intakeMechanism.Spindexer;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.util.PoseEX;

public class CommandMechanism extends BaseMechanism{
    private final Notifier notifier;

    private final ScoreMath scoreMath;
    private double[] dynamicScoringData = new double[]{0,0,0};
    private double[] dynamicPassLeft = new double[]{0,0,0};
    private double[] dynamicPassRight = new double[]{0,0,0};
    private boolean interpolate = false;
    private final double maxAimError = .02;

    public CommandMechanism(Arm arm, Intake intake, Indexer indexer, Spindexer spindexer, Turret turret, Hood hood, Shooter shooter, ClimbElevator climbElevator, SwerveDrive swerveDrive, CommandXboxController commandXboxController){
        super(arm, intake, indexer, spindexer, turret, hood, shooter, climbElevator, swerveDrive, commandXboxController);
        scoreMath = new ScoreMath(swerveDrive, turret.fromSwerveBase);

        turret.setDefaultCommand(turret.reachGoal(()->PoseEX.correctedRotation(dynamicScoringData[2]-swerveDrive.getYaw().getRotations())));

        notifier = new Notifier(this :: commandPeriodic);
        notifier.setName("CommandMechanism Periodic");
        notifier.startPeriodic(.02);
        Runtime.getRuntime().addShutdownHook(new Thread(notifier::close));
    }

    // @Override
    // public boolean readyToShoot(){
    //     return super.readyToShoot()
    //     && Math.abs(dynamicScoringData[2]-swerveDrive.getYaw().getRotations()-turret.getPosition()) < maxAimError;
    // }

    public double mpsTorps(double mps){
        return mps;
    }

    public double rpsTomps(double rps){
        return rps;
    }

    public double hoodRotationsToShootRotations(double rotations){
        return Robot.isSimulation() ? rotations : rotations;
    }

    public double shootRotationsToHoodRotations(double rotations){
        return Robot.isSimulation() ? rotations : rotations;
    }

    public Command shootDefault(Supplier<double[]> dynamicScoringData){
        return shootContinuous(()->shootRotationsToHoodRotations(dynamicScoringData.get()[0])
            ,()->mpsTorps(dynamicScoringData.get()[1])
            ,()->PoseEX.correctedRotation(dynamicScoringData.get()[2]-swerveDrive.getYaw().getRotations()));
    }

    public Command shootDynamic() {
        return shootDefault(()->dynamicScoringData);
    }

    public Command shootStatic() {
        return Commands.parallel(swerveDrive.brake()
            ,shootDefault(()->dynamicScoringData));
    }

    public Command shootDynamicNoTurret() {
        return shootDefault(()->dynamicScoringData)
            .alongWith(swerveDrive.pointWhileDrive(()->Rotation2d.fromRotations(dynamicScoringData[2]), xboxController, 5,1,5,1));
    }

    public Command passDynamic(boolean left) {
        return shootDefault(()-> left ? dynamicPassLeft : dynamicPassRight);
    }

    public Command passStatic(boolean left) {
        return Commands.parallel(swerveDrive.brake()
            ,shootDefault(()-> left ? dynamicPassLeft : dynamicPassRight));
    }

    public Command passDynamicNoTurret(boolean left) {
        return shootDefault(()-> left ? dynamicPassLeft : dynamicPassRight)
            .alongWith(swerveDrive.pointWhileDrive(()->Rotation2d.fromRotations(left ? dynamicPassLeft[2] : dynamicPassRight[2]), xboxController, 5,1,5,1));
    }

    public void commandPeriodic(){
        dynamicScoringData = scoreMath.dynamicScore(GameData.getHubPose3d(), interpolate && !Robot.isSimulation()
            ,hoodRotationsToShootRotations(.055), hoodRotationsToShootRotations(.125), rpsTomps(8));

        dynamicPassLeft = scoreMath.dynamicScore(GameData.getPassPose3d(true), interpolate && !Robot.isSimulation()
            ,hoodRotationsToShootRotations(.055), hoodRotationsToShootRotations(.125), rpsTomps(10));
        dynamicPassRight = scoreMath.dynamicScore(GameData.getPassPose3d(false), interpolate && !Robot.isSimulation()
            ,hoodRotationsToShootRotations(.055), hoodRotationsToShootRotations(.125), rpsTomps(10));
    }
}
