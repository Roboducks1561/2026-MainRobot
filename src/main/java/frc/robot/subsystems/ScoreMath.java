package frc.robot.subsystems;

import java.util.function.DoubleConsumer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.util.MultiLinearInterpolator;
import frc.robot.util.PoseEX;
import frc.robot.util.SendableConsumer;
import frc.robot.util.Vector2;

public class ScoreMath {

    private final SwerveDrive swerveDrive;
    private final Transform3d turretTransform;
    public ScoreMath(SwerveDrive swerveDrive, Transform3d turretTransform){
        this.swerveDrive = swerveDrive;
        this.turretTransform = turretTransform;
        SendableConsumer.checker(
            SendableConsumer.createSendableChooser("interpolationTuning", new String[]{"hood additional", "shooter divisor"}, new double[]{0.0,1.0})
        ,new DoubleConsumer[]{
            (i)->{additional = i;},
            (i)->{divisor = i;}
        });
    }

    /**
     * 
     * @param scorePose
     * @returns a double array with {Shoot rotations, velocity meters per second, target direction rotations}
     */
    public double[] staticScore(Pose3d scorePose, boolean interpolate, double minShootAngle, double maxShootAngle, double maxVelocityMPS){
        Pose3d turretPose = new Pose3d(swerveDrive.getPose()).transformBy(turretTransform);
        if (interpolate){
            return interpolate(turretPose, scorePose);
        }
        return shootCalculations(turretPose, scorePose, minShootAngle, maxShootAngle, maxVelocityMPS);
    }

    /**
     * 
     * @param scorePose
     * @returns a double array with {Shoot rotations, velocity meters per second, target direction rotations}
     */
    public double[] dynamicScore(Pose3d scorePose, boolean interpolate, double minShootAngle, double maxShootAngle, double maxVelocityMPS){
        //TODO timeTillTarget is an educated guess, please fix
        //Done by finding the time it takes to hit the ground at 2.6 meters, multiplying by proportion I think it will actually travel (Because it hits hub before ground)
        double timeTillTarget = .728*1.8;
        if (interpolate){
            timeTillTarget = .728 * 1.4;
        }

        //TODO the acceleration might not be working right, check this first if missing, .2 is mostly a guess too
        ChassisSpeeds swerveSpeeds = swerveDrive.getSpeeds().plus(swerveDrive.getAcceleration().times(.2));
        Pose3d turretPose = new Pose3d(swerveDrive.getPose()).transformBy(new Transform3d(swerveSpeeds.vxMetersPerSecond * timeTillTarget, swerveSpeeds.vyMetersPerSecond * timeTillTarget,0,new Rotation3d())).transformBy(turretTransform);
        if (interpolate){
            return interpolate(turretPose, scorePose);
        }
        return shootCalculations(turretPose, scorePose, minShootAngle, maxShootAngle, maxVelocityMPS);
    }

    public double[] shootCalculations(Pose3d turretPose, Pose3d scorePose, double minShootAngle, double maxShootAngle, double maxVelocityMPS){
        double heightDif = scorePose.getZ() - turretPose.getZ();
        double dist = PoseEX.getDistanceFromPoseMeters(turretPose.toPose2d(),scorePose.toPose2d());

        Vector2 point1 = new Vector2(0, 0);
        Vector2 point2 = new Vector2(dist, heightDif);

        double[] answers = new double[3];
        double[] hoodSpeedVals = hoodSpeedCalc(minShootAngle, maxShootAngle, maxVelocityMPS, point1, point2);
        answers[0] = hoodSpeedVals[0];
        answers[1] = hoodSpeedVals[1];

        double targetRotation = targetRotation(turretPose.toPose2d(), scorePose.toPose2d());
        answers[2] = targetRotation;
        return answers;
    }

    /**
     * in pitch, so 0 is straight up, and .25 is straight out
     * 
     * https://en.wikipedia.org/wiki/Projectile_motion
     * This is the source I used to get angle given velocity, solving for both semi optimally
     * 
     * Keep in mind, this finds the angle as in 0 is straight up, and .25 is straight out
     * @param minShootAngleRotations
     * @param maxShootAngleRotations
     * @param maxVelocity
     * @param point1
     * @param point2
     * @return
     */
    public double[] hoodSpeedCalc(double minShootAngleRotations, double maxShootAngleRotations, double maxVelocity, Vector2 point1, Vector2 point2){
        //TODO worst method in class, please fix
        try {
            double bestVelocity = 0;
            double bestPivot = 0;
            double closestY = 0;

            double targetMaxY = 2.6;

            double dx = point2.x - point1.x;
            double dy = point2.y - point1.y;
            double g = 9.81;
            if (dx == 0){
                return new double[]{0,0};
            }
            for (double v = 0; v < maxVelocity; v+=.05){
                double angle = Math.atan((v*v +Math.sqrt(v*v*v*v - g*(g*dx*dx + 2*dy*v*v)))/(g*dx));
                
                double fixedAngle = .25-Units.radiansToRotations(angle);
                if (fixedAngle > maxShootAngleRotations || fixedAngle < minShootAngleRotations){
                    continue;
                }

                //This next section is purely selection logic
                double t = Math.sin(angle)*v/g;
                double maxY = Math.sin(angle)*v*t - (g/2)*t*t;

                if (Math.abs(targetMaxY - maxY) < Math.abs(targetMaxY - closestY)){
                    bestPivot = fixedAngle;
                    bestVelocity = v;
                    closestY = maxY;
                }
            }
            return new double[]{bestPivot,bestVelocity};

            
            
        } catch (Exception e) {
            return new double[]{0,0};
        }
    }

    /**
     * Use something such as new Pose3d(swerveDrive.getPose()).transformBy(turret.fromSwerveBase);
     * for turret pose. Score pose stays constant as hub
     * @param turretPose
     * @param scorePose
     * @return
     */
    public double targetRotation(Pose2d turretPose, Pose2d scorePose){
        double targetRotation = PoseEX.getPoseAngle(turretPose, scorePose).getRotations();
        return targetRotation;
    }



    private double divisor = .98;
    private double additional = .03;
    private MultiLinearInterpolator distToSpeedAndAngle = new MultiLinearInterpolator(new double[][]
        {
            //Distance meters, Pivot rotations, velocity
            {1.5426,0.0,68.6}
            ,{1.818,0.007,70}
            ,{2.008,0.01,70}
            ,{2.2,0.015,73}
            ,{2.376,0.015,73}
            ,{2.573,0.02,73}
            ,{2.772,0.027,78}
            ,{3.03,0.031,79}
            ,{3.241,0.031,79}
            ,{3.465,0.038,82}
            ,{3.82,0.043,88}
            ,{4.095,0.046,90}
            ,{4.365,0.048,91}
            ,{4.751,0.06,97}
            ,{5.3,0.06,97}
            // ,{1.3852,0.005+additional,70}
            // ,{1.903,.018+additional,73}
            // ,{2.77,0.034+additional,82}
            // ,{3.22,0.038+additional,85}
            // ,{3.717,0.045+additional,83}
            // ,{4.34,0.044+additional,87}
            // ,{4.623,0.044+additional,91}
            // ,{5.76, 0.06+additional, 91}
            // ,{6, 0.06+additional, 91}
            // ,{6, 0.06+additional, 91}
        }
    );

    /**
     * Keep in mind, this DIFFERS from shootCalculations because you do not need to convert from targets to rps and actual pivot rotations.
     * This does so by default!!!
     * @param turretPose
     * @param scorePose
     * @return
     */
    public double[] interpolate(Pose3d turretPose, Pose3d scorePose){
        double dist = PoseEX.getDistanceFromPoseMeters(turretPose.toPose2d(), scorePose.toPose2d());
        double[] interpolated = distToSpeedAndAngle.get(dist);
        return new double[]{interpolated[0] + additional, interpolated[1]/divisor, targetRotation(turretPose.toPose2d(), scorePose.toPose2d())};
    }
}
