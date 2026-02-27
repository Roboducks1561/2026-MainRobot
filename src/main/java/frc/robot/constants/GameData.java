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
    
    public static final double fieldSizeX = Units.inchesToMeters(651.2);
    public static final double fieldSizeY = Units.inchesToMeters(317.7);

    public static final double funnelHeight = 1.515;
    public static final double rimHeight = 1.8288;
    public static final double rimRadius = .612;

    public static final Pose3d scorePose3d = new Pose3d(4.637,fieldSizeY/2 + .15,funnelHeight, new Rotation3d());
    public static final Pose2d scorePose2d = scorePose3d.toPose2d();

    public static final Pose2d defaultShootPosition = new Pose2d(3.7,0.662,Rotation2d.fromDegrees(66));

    public static final Pose3d leftPassPose3d = new Pose3d(1,fieldSizeY-1,0,new Rotation3d());
    public static final Pose3d rightPassPose3d = new Pose3d(1,1,0,new Rotation3d());

    public static final Pose2d leftPassPose2d = leftPassPose3d.toPose2d();
    public static final Pose2d rightPassPose2d = rightPassPose3d.toPose2d();

    public static final Pose2d depotPose = new Pose2d(.3,6,new Rotation2d());
    public static final Pose2d outpostPose = new Pose2d(.3,.66,new Rotation2d());

    public static final Pose2d trenchPose = new Pose2d(4.626, .634, new Rotation2d());
    public static final Pose2d bumpPose = new Pose2d(4.626, 2.196, new Rotation2d());

    public static final Pose2d towerPose = new Pose2d(1, 3.745, new Rotation2d());
    public static final Pose2d towerPoseLeft = new Pose2d(1, 4, new Rotation2d());
    public static final Pose2d towerPoseRight = new Pose2d(1, 3.49, new Rotation2d());

    public static final Pose2d[] aprilTagsPose2d;
    public static final Pose3d[] aprilTagsPose3d;

    public static final Pose2d[] dangerousPoses;

    static{
        aprilTagsPose2d = new Pose2d[LimelightConstants.K_TAG_LAYOUT.getTags().size()];
        aprilTagsPose3d = new Pose3d[LimelightConstants.K_TAG_LAYOUT.getTags().size()];
        int i = 0;
        for (AprilTag tag : LimelightConstants.K_TAG_LAYOUT.getTags()){
            aprilTagsPose2d[i] = tag.pose.toPose2d();
            aprilTagsPose3d[i] = tag.pose;
            i++;
        }

        dangerousPoses = new Pose2d[]{
            getTrenchPose(false)
            ,getTrenchPose(true)
            ,PoseEX.pose180(getTrenchPose(false))
            ,PoseEX.pose180(getTrenchPose(true))
            ,getBumpPose(false)
            ,getBumpPose(true)
            ,PoseEX.pose180(getBumpPose(false))
            ,PoseEX.pose180(getBumpPose(true))
        };
    }

    public static Pose3d getHubPose3d(){
        if (isRed.getAsBoolean()){
            return PoseEX.pose180(scorePose3d);
        }
        return scorePose3d;
    }

    public static Pose2d getHubPose2d(){
        if (isRed.getAsBoolean()){
            return PoseEX.pose180(scorePose2d);
        }
        return scorePose2d;
    }

    public static Pose3d getPassPose3d(boolean left){
        Pose3d pose = rightPassPose3d;
        if (left){
            pose = leftPassPose3d;
        }
        if (isRed.getAsBoolean()){
            return PoseEX.pose180(pose);
        }
        return pose;
    }

    public static Pose2d getPassPose2d(boolean left){
        Pose2d pose = rightPassPose2d;
        if (left){
            pose = leftPassPose2d;
        }
        if (isRed.getAsBoolean()){
            return PoseEX.pose180(pose);
        }
        return pose;
    }

    public static Pose2d getDefaultShootPose2d(boolean left){
        Pose2d pose = defaultShootPosition;
        if (left){
            pose = new Pose2d(defaultShootPosition.getX(), fieldSizeY-defaultShootPosition.getY(), defaultShootPosition.getRotation().times(-1));
        }
        if (isRed.getAsBoolean()){
            return PoseEX.pose180(pose);
        }
        return pose;
    }

    public static Pose2d getDepotPose(){
        if (isRed.getAsBoolean()){
            return PoseEX.pose180(depotPose);
        }
        return depotPose;
    }

    public static Pose2d getOutpostPose(){
        if (isRed.getAsBoolean()){
            return PoseEX.pose180(outpostPose);
        }
        return outpostPose;
    }

    public static Pose2d getBumpPose(boolean left){
        Pose2d pose = bumpPose;
        if (left){
            pose = new Pose2d(pose.getX(), fieldSizeY - pose.getY(), new Rotation2d());
        }
        if (isRed.getAsBoolean()){
            return PoseEX.pose180(pose);
        }
        return pose;
    }

    public static Pose2d getTrenchPose(boolean left){
        Pose2d pose = trenchPose;
        if (left){
            pose = new Pose2d(pose.getX(), fieldSizeY - pose.getY(), new Rotation2d());
        }
        if (isRed.getAsBoolean()){
            return PoseEX.pose180(pose);
        }
        return pose;
    }

    /**
     * The integer would be 1 for left, 2 for middle, 3 for right
     * @param align
     * @return
     */
    public static Pose2d getTowerPose(int align){
        Pose2d pose = towerPoseLeft;

        if (align == 2){
            pose = towerPose;
        }
        if (align == 3){
            pose = towerPoseRight;
        }

        if (isRed.getAsBoolean()){
            return PoseEX.pose180(pose);
        }
        return pose;
    }

    public static Pose2d centerField(){
        return new Pose2d(fieldSizeX/2, fieldSizeY/2, new Rotation2d());
    }

    public static Pose2d getAprilTagPose2d(int id){
        int fixedNum = Math.max(Math.min(id,22),1)-1;
        return aprilTagsPose2d[fixedNum];
    }

    public static Pose3d getAprilTagPose3d(int id){
        int fixedNum = Math.max(Math.min(id,22),1)-1;
        return aprilTagsPose3d[fixedNum];
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
