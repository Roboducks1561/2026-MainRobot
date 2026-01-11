// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.defaultSystems.vision.objectDetection;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

//TODO imperfect, but close
public class ObjectDetection extends SubsystemBase {
    
    private String limelightName;
    private Transform3d limelightTransform;
    private Supplier<Pose2d> robotPose;
    private ObjectDetectionIO objectDetectionIO;

    private Optional<Pose2d> cachedObjectPose = null;

    private double goalHeightMeters = Units.inchesToMeters(9);

    private final NetworkTable robot = NetworkTableInstance.getDefault().getTable("Robot");
    private final NetworkTable objectTable = robot.getSubTable("ObjectDetection");

    private final StructPublisher<Pose2d> objectPublisher = objectTable
        .getStructTopic("ObjectPose", Pose2d.struct).publish();;

    /** Creates a new ObjectDetection. */
    public ObjectDetection(String limelightName, Transform3d limelightTransform, Supplier<Pose2d> robotPose) {
        this.limelightName = limelightName;
        this.limelightTransform = limelightTransform;
        this.robotPose = robotPose;

        if (Robot.isSimulation()){
            objectDetectionIO = new ObjectDetectionSim(limelightTransform, VecBuilder.fill(63.3,49.7), robotPose);
        }else{
            objectDetectionIO = new LimelightObjectDetection(limelightName);
        }
    }
    
    public Rotation2d getHorizontalRotationFromPiece(){
        return objectDetectionIO.getHorizontalRotationFromTarget();
    }
    
    public Rotation2d getVerticalRotationFromPiece(){
        return objectDetectionIO.getVerticalRotationFromPiece();
    }

    public boolean isPiecePresent(){
        return objectDetectionIO.isPiecePresent();
    }

    public double getDistanceFromPieceVertical(){
        if (isPiecePresent()){
            double targetOffsetAngle_Vertical = getVerticalRotationFromPiece().getRadians();
            // how many degrees back is your limelight rotated from perfectly vertical?
            double limelightMountAngleDegrees = limelightTransform.getRotation().getY();

            // distance from the center of the Limelight lens to the floor
            double limelightLensHeightMeters = limelightTransform.getZ();

            // distance from the target to the floor
            double goalHeightMeters = this.goalHeightMeters;

            double angleToGoalRadians = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
            return  (limelightLensHeightMeters-goalHeightMeters) / -(Math.sin(angleToGoalRadians));
        }
        return 0;
    }

    public double getDistanceFromPieceHorizontal(){
        if (isPiecePresent()){
            double targetOffsetAngle_Horizontal = getHorizontalRotationFromPiece().getRadians();

            // what is the yaw of your limelight
            double limelightMountAngleRadians = limelightTransform.getRotation().getZ(); 

            double angleToGoalRadians = limelightMountAngleRadians + targetOffsetAngle_Horizontal;
            return getDistanceFromPieceVertical()*Math.tan(angleToGoalRadians);
        }
        return 0;
    }

    /**
     * this method takes alot to compute, and is being called periodically anyway, so I cache the result and you can 
     * get it from "getCachedObjectPose()" unless you need a very low latency result for some reason
     * @return
     */
    public Optional<Pose2d> getPiecePose(){
        if (isPiecePresent()){
            Pose3d initialPose = new Pose3d(robotPose.get()).transformBy(limelightTransform);
            double initialPitch = initialPose.getRotation().getY(); // Initial pose's pitch in radians
            double pitch = Math.toRadians(getVerticalRotationFromPiece().getDegrees()); // Target pitch in radians
            double yaw = Math.toRadians(getHorizontalRotationFromPiece().getDegrees())*-1; // Target yaw in radians
        
            // Calculate total pitch and yaw
            double totalPitch = initialPitch + pitch;
            double totalYaw = initialPose.getRotation().getZ() + yaw;
        
            // Calculate direction vector components
            double dx = Math.cos(totalPitch) * Math.cos(totalYaw);
            double dy = Math.cos(totalPitch) * Math.sin(totalYaw);
            double dz = Math.sin(totalPitch);

            // Initial position
            double x0 = initialPose.getX();
            double y0 = initialPose.getY();
            double z0 = initialPose.getZ();
        
            // Calculate distance to the floor
            double t = z0 / -dz;
        
            // Calculate new position
            double newX = x0 + t * dx;
            double newY = y0 + t * dy;

            double newZ = goalHeightMeters;
        
            // Create new orientation (pitch and yaw)
            Rotation3d newRotation = new Rotation3d(totalPitch, 0, totalYaw);
        
            // Create and return new pose
            cachedObjectPose = Optional.of(new Pose3d(newX, newY, newZ, newRotation).toPose2d());
            return cachedObjectPose;
            // Pose2d pose = robotPose.get();
            // double x = getDistanceFromPieceVertical();
            // double y = getDistanceFromPieceHorizontal();
            // double xn = x*Math.cos(pose.getRotation().getRadians())- y*Math.sin(pose.getRotation().getRadians());
            // double yn = x*Math.sin(pose.getRotation().getRadians())+ y*Math.cos(pose.getRotation().getRadians());
            // return new Pose2d(xn+pose.getX(),yn+pose.getY(),Rotation2d.fromDegrees(PoseEX.getYawFromPose(pose, new Pose2d(xn+pose.getX(),yn+pose.getY(), new Rotation2d())).getDegrees()+robotPose.get().getRotation().getDegrees()));
        }
        cachedObjectPose = Optional.empty();
        return cachedObjectPose;
    }

    public Optional<Pose2d> getCachedObjectPose(){
        return cachedObjectPose;
    }

    @Override
    public void periodic() {
        Optional<Pose2d> piecePose = getPiecePose();
        if (piecePose.isPresent()){
            objectPublisher.accept(piecePose.get());
        }else{
            objectPublisher.accept(robotPose.get());
        }
    }
}
