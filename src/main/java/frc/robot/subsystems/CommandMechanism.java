package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.GameData;
import frc.robot.constants.HoodConstants;
import frc.robot.subsystems.TurretMechanism.Hood;
import frc.robot.subsystems.TurretMechanism.Shooter;
import frc.robot.subsystems.TurretMechanism.Turret;
import frc.robot.subsystems.intakeMechanism.Arm;
import frc.robot.subsystems.intakeMechanism.Indexer;
import frc.robot.subsystems.intakeMechanism.Intake;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.util.PoseEX;
import frc.robot.util.Vector2;

public class CommandMechanism extends BaseMechanism{
    private final Notifier notifier;
    private double[] shootingData;
    public CommandMechanism(Arm arm, Intake intake, Indexer indexer, Turret turret, Hood hood, Shooter shooter, SwerveDrive swerveDrive){
        super(arm, intake, indexer, turret, hood, shooter, swerveDrive);
        notifier = new Notifier(this :: scoringPeriodic);
        notifier.setName("Scoring Periodic");
        notifier.startPeriodic(.02);
        Runtime.getRuntime().addShutdownHook(new Thread(notifier::close));
    }

    public double[] hoodSpeedCalc(Vector2 point1, Vector2 point2){
        try {
            double dx = point2.x - point1.x;
            double dy = point2.y - point1.y;

            double lastBestTheta = 0;
            double lastBestVelocity = 0;
            
            for (double i = hood.invertInterpolation(Units.radiansToRotations(HoodConstants.MIN_HOOD_ANGLE_RAD)); i > hood.invertInterpolation(Units.radiansToRotations(HoodConstants.MAX_HOOD_ANGLE_RAD)); i-=.005){
                double theta = Units.rotationsToRadians(i);
                double denom = 2*Math.pow(Math.cos(theta),2) * (dx * Math.tan(theta) - dy);
                if (denom <= 0){
                    continue;
                }
                double velocity = Math.sqrt(9.81 * dx * dx / denom);
                if (velocity > 7){
                    continue;
                }

                return new double[]{Units.radiansToRotations(theta), velocity};
            }
            System.out.println("Failed");
            return new double[]{Units.radiansToRotations(lastBestTheta), lastBestVelocity};
        } catch (Exception e) {
            return new double[]{0,0};
        }
    }

    // /**
    //  * returns needed hood position (rotations), and shooter speed (rpm) for current location
    //  * @return
    //  */
    // @Deprecated
    // public double[] calculateScoringPositions(Pose3d scorePose){
    //     Pose3d turretPose = new Pose3d(swerveDrive.getPose()).transformBy(turret.fromSwerveBase);
    //     double dist = PoseEX.getDistanceFromPoseMeters(turretPose.toPose2d(),scorePose.toPose2d());
    //     double heightDif = scorePose.getZ() - turretPose.getZ();

    //     Vector2 point1 = new Vector2(0, 0);
    //     Vector2 point2 = new Vector2(dist, heightDif);

    //     try {
    //         double a = (point1.y*point2.x - point2.y*point1.x)/(Math.pow(point1.x,2)*point2.x - Math.pow(point2.x,2) * point1.x);
    //         double b = (point2.y*Math.pow(point1.x,2) - point1.y*Math.pow(point2.x,2))/(Math.pow(point1.x,2)*point2.x - Math.pow(point2.x,2) * point1.x);

    //         double theta = Math.atan(b);

    //         double n = a*Math.pow(-b/(2*a),2) - Math.pow(b,2)/(2*a);
    //         double velocity = Math.sqrt(-4*-9.81*n)/Math.sin(theta);

    //         // if (hood.linearInterpolation(Units.radiansToRotations(theta)) > Units.radiansToRotations(HoodConstants.MAX_HOOD_ANGLE_RAD)){                
    //         //     theta = hood.invertInterpolation(Units.radiansToRotations(HoodConstants.MAX_HOOD_ANGLE_RAD));
    //         //     velocity = Math.sqrt((9.81*Math.pow(point2.x,2))
    //         //     /((2*Math.pow(Math.cos(theta), 2))*(point2.x*Math.tan(theta)-point2.y)));
    //         // }
    //         // if (hood.linearInterpolation(Units.radiansToRotations(theta)) < Units.radiansToRotations(HoodConstants.MIN_HOOD_ANGLE_RAD)){
    //         //     theta = hood.invertInterpolation(Units.radiansToRotations(HoodConstants.MIN_HOOD_ANGLE_RAD));
    //         //     velocity = Math.sqrt((9.81*Math.pow(point2.x,2))
    //         //     /((2*Math.pow(Math.cos(theta), 2))*(point2.x*Math.tan(theta)-point2.y)));
    //         // }

    //         return new double[]{Units.radiansToRotations(theta), velocity};
    //     } catch (Exception e) {
    //         return new double[]{0,0};
    //     }
    // }

    public double[] turretCalc(Pose2d turretPose, Pose2d scorePose){
        double targetRotation = PoseEX.getPoseAngle(turretPose, scorePose).getRotations();
        double swerveDriveRotation = swerveDrive.getPose().getRotation().getRotations();

        return new double[]{targetRotation, PoseEX.correctedRotation(targetRotation - swerveDriveRotation)};
    }

    public double[] calculateStaticScoring(Pose3d scorePose){

        double[] answers = new double[4];
        Pose3d turretPose = new Pose3d(swerveDrive.getPose()).transformBy(turret.fromSwerveBase);
        double heightDif = scorePose.getZ() - turretPose.getZ();
        double dist = PoseEX.getDistanceFromPoseMeters(turretPose.toPose2d(),scorePose.toPose2d());

        Vector2 point1 = new Vector2(0, 0);
        Vector2 point2 = new Vector2(dist, heightDif);


        double[] hoodSpeedVals = hoodSpeedCalc(point1, point2);
        answers[0] = hoodSpeedVals[0];
        answers[1] = hoodSpeedVals[1];

        double[] turretVals = turretCalc(turretPose.toPose2d(), scorePose.toPose2d());
        answers[2] = turretVals[0];
        answers[3] = turretVals[1];
        return answers;
    }


    public double[] calculateMovingScoring(Pose3d scorePose){

        double[] answers = new double[]{0,0,0,0};

        double heightDif = scorePose.getZ() - turret.fromSwerveBase.getZ();
        double timeTillTarget = 10;

        ChassisSpeeds swerveSpeeds = swerveDrive.getSpeeds();//.plus(swerveDrive.getAcceleration().times(timeTillTarget));
        Pose3d turretPose = new Pose3d(swerveDrive.getPose()).transformBy(new Transform3d(swerveSpeeds.vxMetersPerSecond * timeTillTarget, swerveSpeeds.vyMetersPerSecond * timeTillTarget,0,new Rotation3d())).transformBy(turret.fromSwerveBase);
        double dist = PoseEX.getDistanceFromPoseMeters(turretPose.toPose2d(),scorePose.toPose2d());
        
        Vector2 point1 = new Vector2(0, 0);
        Vector2 point2 = new Vector2(dist, heightDif);

        double[] hoodSpeedVals = hoodSpeedCalc(point1, point2);
        answers[0] = hoodSpeedVals[0];
        answers[1] = hoodSpeedVals[1];

        double[] turretVals = turretCalc(turretPose.toPose2d(), scorePose.toPose2d());
        answers[2] = turretVals[0];
        answers[3] = turretVals[1];
        return answers;
    }

    /**
     * returns needed hood position (rotations), and shooter speed (rpm) for current location
     * @return
     */
    public double[] testScoringPositions(double dist, double height){
        Pose3d turretPose = new Pose3d(swerveDrive.getPose()).transformBy(turret.fromSwerveBase);
        double heightDif = height - turretPose.getZ();

        Vector2 point1 = new Vector2(0, 0);
        Vector2 point2 = new Vector2(dist, heightDif);

        return hoodSpeedCalc(point1, point2);
    }

    public Command autoScore(Pose3d scorePose){
        return shoot(
            ()->shootingData[2]
            ,()->shootingData[3]
            ,()->shootingData[0]
            ,()->shootingData[1]
        );
    }

    public Command scoreWhileMoving(Pose3d scorePose){
        return shoot(
            ()->shootingData[3]
            ,()->shootingData[0]
            ,()->shootingData[1]
        );
    }

    public Command testScore(double dist, double height){
        return shoot(
            ()->0
            ,()->testScoringPositions(dist,height)[0]
            ,()->testScoringPositions(dist,height)[1]
        );
    }

    public void scoringPeriodic(){
        shootingData = calculateStaticScoring(GameData.scorePose3d);
    }
}
