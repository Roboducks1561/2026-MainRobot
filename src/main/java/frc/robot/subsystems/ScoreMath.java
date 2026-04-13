package frc.robot.subsystems;

import java.util.function.DoubleConsumer;

import edu.wpi.first.hal.simulation.RoboRioDataJNI;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
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
        
        
        
        // SendableConsumer.checker(
        //     SendableConsumer.createSendableChooser("interpolationTuning", new String[]{"hood additional", "shooter multiplier"}, new double[]{0.0,1})
        // ,new DoubleConsumer[]{
        //     (i)->{additional = i;},
        //     (i)->{divisor = i;}
        // });
    }

    // /**
    //  * 
    //  * @param scorePose
    //  * @returns a double array with {Shoot rotations, velocity meters per second, target direction rotations}
    //  */
    // public double[] staticScore(Pose3d scorePose, boolean interpolate, double minShootAngle, double maxShootAngle, double maxVelocityMPS){
    //     Pose3d turretPose = new Pose3d(swerveDrive.getPose()).transformBy(turretTransform);
    //     if (interpolate){
    //         return interpolate(turretPose, scorePose);
    //     }
    //     return shootCalculations(turretPose, scorePose, minShootAngle, maxShootAngle, maxVelocityMPS);
    // }

    // /**
    //  * 
    //  * @param scorePose
    //  * @returns a double array with {Shoot rotations, velocity meters per second, target direction rotations}
    //  */
    // public double[] dynamicScore(Pose3d scorePose, boolean interpolate, double minShootAngle, double maxShootAngle, double maxVelocityMPS){
    //     //TODO timeTillTarget is an educated guess, please fix
    //     //Done by finding the time it takes to hit the ground at 2.6 meters, multiplying by proportion I think it will actually travel (Because it hits hub before ground)
    //     double timeTillTarget = .728*1.8;
    //     if (interpolate){
    //         timeTillTarget = .728 * 1.73;
    //     }

    //     //TODO the acceleration might not be working right, check this first if missing, .2 is mostly a guess too
    //     ChassisSpeeds swerveSpeeds = swerveDrive.getSpeeds().plus(swerveDrive.getAcceleration().times(.2));
    //     Pose3d turretPose = new Pose3d(swerveDrive.getPose()).transformBy(new Transform3d(swerveSpeeds.vxMetersPerSecond * timeTillTarget, swerveSpeeds.vyMetersPerSecond * timeTillTarget,0,new Rotation3d())).transformBy(turretTransform);
    //     if (interpolate){
    //         return interpolate(turretPose, scorePose);
    //     }
    //     return shootCalculations(turretPose, scorePose, minShootAngle, maxShootAngle, maxVelocityMPS);
    // }

    // public double[] shootCalculations(Pose3d turretPose, Pose3d scorePose, double minShootAngle, double maxShootAngle, double maxVelocityMPS){
    //     double heightDif = scorePose.getZ() - turretPose.getZ();
    //     double dist = PoseEX.getDistanceFromPoseMeters(turretPose.toPose2d(),scorePose.toPose2d());

    //     Vector2 point1 = new Vector2(0, 0);
    //     Vector2 point2 = new Vector2(dist, heightDif);

    //     double[] answers = new double[3];
    //     double[] hoodSpeedVals = hoodSpeedCalc(minShootAngle, maxShootAngle, maxVelocityMPS, point1, point2);
    //     answers[0] = hoodSpeedVals[0];
    //     answers[1] = hoodSpeedVals[1];

    //     double targetRotation = targetRotation(turretPose.toPose2d(), scorePose.toPose2d());
    //     answers[2] = targetRotation;
    //     return answers;
    // }

    // /**
    //  * in pitch, so 0 is straight up, and .25 is straight out
    //  * 
    //  * https://en.wikipedia.org/wiki/Projectile_motion
    //  * This is the source I used to get angle given velocity, solving for both semi optimally
    //  * 
    //  * Keep in mind, this finds the angle as in 0 is straight up, and .25 is straight out
    //  * @param minShootAngleRotations
    //  * @param maxShootAngleRotations
    //  * @param maxVelocity
    //  * @param point1
    //  * @param point2
    //  * @return
    //  */
    // public double[] hoodSpeedCalc(double minShootAngleRotations, double maxShootAngleRotations, double maxVelocity, Vector2 point1, Vector2 point2){
    //     //TODO worst method in class, please fix
    //     try {
    //         double bestVelocity = 0;
    //         double bestPivot = 0;
    //         double closestY = 0;

    //         double targetMaxY = 2.6;

    //         double dx = point2.x - point1.x;
    //         double dy = point2.y - point1.y;
    //         double g = 9.81;
    //         if (dx == 0){
    //             return new double[]{0,0};
    //         }
    //         for (double v = 0; v < maxVelocity; v+=.05){
    //             double angle = Math.atan((v*v +Math.sqrt(v*v*v*v - g*(g*dx*dx + 2*dy*v*v)))/(g*dx));
                
    //             double fixedAngle = .25-Units.radiansToRotations(angle);
    //             if (fixedAngle > maxShootAngleRotations || fixedAngle < minShootAngleRotations){
    //                 continue;
    //             }

    //             //This next section is purely selection logic
    //             double t = Math.sin(angle)*v/g;
    //             double maxY = Math.sin(angle)*v*t - (g/2)*t*t;

    //             if (Math.abs(targetMaxY - maxY) < Math.abs(targetMaxY - closestY)){
    //                 bestPivot = fixedAngle;
    //                 bestVelocity = v;
    //                 closestY = maxY;
    //             }
    //         }
    //         return new double[]{bestPivot,bestVelocity};

            
            
    //     } catch (Exception e) {
    //         return new double[]{0,0};
    //     }
    // }

    // /**
    //  * 
    //  * @param scorePose
    //  * @returns a double array with {Shoot rotations, velocity meters per second, target direction rotations}
    //  */
    public double[] staticScore(Pose3d scorePose, boolean interpolate, double shootAngle, double maxVelocityMPS){
        Pose3d turretPose = new Pose3d(swerveDrive.getPose()).transformBy(turretTransform);
        if (interpolate){
            return interpolate(turretPose, scorePose);
        }
        return shootCalculations(turretPose, scorePose, shootAngle, maxVelocityMPS);
    }

    /**
     * 
     * @param scorePose
     * @returns a double array with {Shoot rotations, velocity meters per second, target direction rotations}
     */
    public double[] dynamicScore(Pose3d scorePose, boolean interpolate, double shootAngle, double maxVelocityMPS){
        //TODO timeTillTarget is an educated guess, please fix
        //Done by finding the time it takes to hit the ground at 2.6 meters, multiplying by proportion I think it will actually travel (Because it hits hub before ground)
        double timeTillTarget = .728*1.8;
        if (interpolate){
            timeTillTarget = .728 * 1.73;
        }

        //TODO the acceleration might not be working right, check this first if missing, .2 is mostly a guess too
        ChassisSpeeds swerveSpeeds = swerveDrive.getSpeeds().plus(swerveDrive.getAcceleration().times(.2));
        Pose3d turretPose = new Pose3d(swerveDrive.getPose()).transformBy(new Transform3d(swerveSpeeds.vxMetersPerSecond * timeTillTarget, swerveSpeeds.vyMetersPerSecond * timeTillTarget,0,new Rotation3d())).transformBy(turretTransform);
        if (interpolate){
            return interpolate(turretPose, scorePose);
        }
        return shootCalculations(turretPose, scorePose, shootAngle, maxVelocityMPS);
    }

    public double[] shootCalculations(Pose3d turretPose, Pose3d scorePose, double shootAngle, double maxVelocityMPS){
        double heightDif = scorePose.getZ() - turretPose.getZ();
        double dist = PoseEX.getDistanceFromPoseMeters(turretPose.toPose2d(),scorePose.toPose2d());

        Vector2 point1 = new Vector2(0, 0);
        Vector2 point2 = new Vector2(dist, heightDif);

        double[] answers = new double[3];
        double[] hoodSpeedVals = hoodSpeedCalc(shootAngle, maxVelocityMPS, point1, point2);
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
    public double[] hoodSpeedCalc(double shootAngle, double maxVelocity, Vector2 point1, Vector2 point2){
        //TODO worst method in class, please fix
        try {
            double dx = point2.x - point1.x;
            double dy = point2.y - point1.y;
            double g = 9.81;
            if (dx == 0){
                return new double[]{0,0};
            }
            for (double v = 0; v < maxVelocity; v+=.05){
                double angle = Math.atan((v*v +Math.sqrt(v*v*v*v - g*(g*dx*dx + 2*dy*v*v)))/(g*dx));
                
                double fixedAngle = .25-Units.radiansToRotations(angle);
                if (fixedAngle > shootAngle-.02 && fixedAngle < shootAngle+.02){
                    return new double[]{0,v};
                }
            }
            return new double[]{0,0};
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



    public static double divisor = .90;
    private double additional = 0.00;

    //tortured poets dept is .98
    //
    private MultiLinearInterpolator distToSpeedAndAngle = new MultiLinearInterpolator(new double[][]
        {
            {0, 0.0, 0},
            {1.89999, 0.0, 0},
            {1.9, 0.0, 46.8},
            {2.02, 0.0, 50},
            {2.15, 0.0, 49},
            {2.3, 0.0, 51},
            {2.45, 0.0, 53},
            {2.6, 0.0, 54},
            {2.75, 0.0, 55},
            {3, 0.0, 56},
            {3.14, 0.0, 57},
            {3.27, 0.0, 60},
            {3.43, 0.0, 63},
            {3.658, 0.0, 65},
            {3.88, 0.0, 67},
            {4.08, 0.0, 72},
            {4.2, 0.0, 75},
            {4.4, 0.0, 78},
            {6, 0.0, 78},
            // {4.08, 0.0, 72},

            // {3.6, 0.0, 66},
            // {3.75, 0.0, 69},
            // {3.9, 0.0, 72},
            // {4.2, 0.0, 75},
            // {8, 0.0, 80},
            // {3.69, 0.0, 56},
            // {4.01, 0.0, 58},
            // {4.29, 0.0, 62},
            // {4.6, 0.0, 68},

            // {2.57, 0.0, 49},
            // {2.91, 0.0, 50},
            // {3.09, 0.0, 53},
            // {3.4, 0.0, 55},
            // {3.69, 0.0, 56},
            // {4.01, 0.0, 58},
            // {4.29, 0.0, 62},
            // {4.6, 0.0, 68},
            // {6, 0.0, 80}


            // {1.22, 0.0, 50},
            // {1.69, 0.01, 51},
            // {2, 0.01, 52},
            // {2.34, 0.03, 49},
            // {2.72, 0.03, 50},
            // {3.165, 0.04, 52},
            // {3.31, 0.05, 54},
            // {3.54, 0.05, 58},
            // {3.786, 0.05, 62},
            // {4.11, 0.05, 63},
            // {4.7, 0.05, 75},


            //Distance meters, Pivot rotations, velocity
            // {1.2715, 0.0, 60}
            // ,{1.57, 0.01, 61}
            // ,{1.974, 0.02, 62}
            // ,{1.974, 0.02, 62}
            // ,{0, 0, 0}
            // {1.492,0.01,57}
            // ,{1.922,0.015,60}
            // ,{2.315,0.022,62}
            // ,{2.58,0.026,64}
            // ,{2.905,0.033,68}
            // ,{3.19,0.037,70}
            // ,{3.65,0.04+.0043,75}
            // ,{4.24,0.05+.0043,77}
            // ,{4.6,0.055+.0043,80}
            // ,{5.1,0.059+.0043,82/0.94}
            // ,{5.76,0.061+.0043,84}
            // ,{100,0.061+.0043,84}
            // ,{3.77,0.048,70}
            // ,{4.14,0.053,72}
            // ,{4.48,0.061,74}
            // ,{4.8,0.061,75}
            // ,{5.1,0.065,77}
            // ,{5.4,0.067,82}
            // ,{5.7,0.08,84}
            // ,{100,0.08,84}
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
        return new double[]{interpolated[0] + additional, interpolated[1]*divisor, targetRotation(turretPose.toPose2d(), scorePose.toPose2d())};
    }
}
