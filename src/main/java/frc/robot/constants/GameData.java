package frc.robot.constants;

import java.util.function.BooleanSupplier;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.util.PoseEX;

public class GameData {

    public static final BooleanSupplier isRed = ()-> DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red;
    
    public static final double fieldSizeX = Units.feetToMeters(54);
    public static final double fieldSizeY = Units.feetToMeters(27);

    public static final double funnelHeight = 1.994;
    public static final double rimHeight = Units.inchesToMeters(25.997 + 30);
    public static final double rimRadius = Units.inchesToMeters(38.7);

    // new Pose3d(fieldSizeX/2, fieldSizeY/2, funnelHeight, new Rotation3d());
    public static final Pose3d scorePose3d = new Pose3d(4.15,2.25,.8382, new Rotation3d());
    public static final Pose2d scorePose2d = scorePose3d.toPose2d();
    
    

    public static final Pose2d[] aprilTagsPose2d;
    public static final Pose3d[] aprilTagsPose3d;

    public static final Pose2d[] redCargoPoses = new Pose2d[]{
        new Pose2d(fieldSizeX/2 - Units.inchesToMeters(25.910), fieldSizeY/2 - Units.inchesToMeters(150.79), new Rotation2d()),
        new Pose2d(fieldSizeX/2 - Units.inchesToMeters(124.946), fieldSizeY/2 - Units.inchesToMeters(88.303), new Rotation2d()),
        new Pose2d(fieldSizeX/2 - Units.inchesToMeters(129.396), fieldSizeY/2 + Units.inchesToMeters(81.643), new Rotation2d()),
        new Pose2d(fieldSizeX/2 - Units.inchesToMeters(33.767), fieldSizeY/2 + Units.inchesToMeters(149.227), new Rotation2d()),
        new Pose2d(fieldSizeX/2 + Units.inchesToMeters(149.227), fieldSizeY/2 + Units.inchesToMeters(33.767), new Rotation2d()),
        new Pose2d(fieldSizeX/2 + Units.inchesToMeters(88.303), fieldSizeY/2 - Units.inchesToMeters(124.946), new Rotation2d())
    };

    public static Pose2d redFeederPose = new Pose2d(fieldSizeX/2 - Units.inchesToMeters(282.080-20), fieldSizeY/2 - Units.inchesToMeters(117.725-20), new Rotation2d());
    static{
        aprilTagsPose2d = new Pose2d[LimelightConstants.K_TAG_LAYOUT.getTags().size()];
        aprilTagsPose3d = new Pose3d[LimelightConstants.K_TAG_LAYOUT.getTags().size()];
        int i = 0;
        for (AprilTag tag : LimelightConstants.K_TAG_LAYOUT.getTags()){
            aprilTagsPose2d[i] = tag.pose.toPose2d();
            aprilTagsPose3d[i] = tag.pose;
            i++;
        }
    }

    public static Pose2d getAprilTagPose2d(int id){
        int fixedNum = Math.max(Math.min(id,22),1)-1;
        return aprilTagsPose2d[fixedNum];
    }

    public static Pose3d getAprilTagPose3d(int id){
        int fixedNum = Math.max(Math.min(id,22),1)-1;
        return aprilTagsPose3d[fixedNum];
    }

    public static Pose2d getCargoPose(int num, boolean red){
        int clamped = MathUtil.clamp(num, 1, 6)-1;
        if (red){
            return redCargoPoses[clamped];
        }
        return PoseEX.pose180(redCargoPoses[clamped]);
    }

    public static Pose2d getFeederPose(boolean red){
        if (red){
            return redFeederPose;
        }
        return PoseEX.pose180(redFeederPose);
    }
    /**
     * does not return a rotation, rotates around the 0,0 of poses
     * @param pose
     * @param angleRadians
     * @return
     */
    private static Pose2d rotate(Pose2d pose, double angleRadians) {
        double cosTheta = Math.cos(angleRadians);
        double sinTheta = Math.sin(angleRadians);
        double newX = pose.getX() * cosTheta - pose.getY() * sinTheta;
        double newY = pose.getX() * sinTheta + pose.getY() * cosTheta;
        return new Pose2d(newX, newY, pose.getRotation());
    }

    
}
