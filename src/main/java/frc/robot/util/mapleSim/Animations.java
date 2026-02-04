package frc.robot.util.mapleSim;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.function.Predicate;

import org.dyn4j.geometry.Rotation;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.Notifier;

public class Animations {

    private final NetworkTable baseTable;
    private final Notifier notifier;
    private final StructArrayPublisher<Translation3d> flyingObjectsPublisher;
    private final ArrayList<Pair<Translation3d, Translation3d>> flyingObjects;

    public Animations(){
        baseTable = NetworkTableInstance.getDefault().getTable("Animations");

        flyingObjectsPublisher = baseTable.getStructArrayTopic("FlyingObjects", Translation3d.struct).publish();
        flyingObjects = new ArrayList<>();

        notifier = new Notifier(this :: periodic);
        notifier.setName("Animations Periodic");
        notifier.startPeriodic(.02);
        Runtime.getRuntime().addShutdownHook(new Thread(notifier::close));
    }

    /**
     * 
     * @param initPose lets say for a turret might be center of turret
     * @param vector The vector field relative of velocity, in m/s
     */
    public void addFlyingObject(Translation3d initPose, Translation3d vector){
        flyingObjects.add(new Pair<Translation3d,Translation3d>(initPose, vector));  
    }


    public void addFlyingObject(Pose2d swervePose, Transform3d shooterPosition, ChassisSpeeds localChassisSpeeds, double launchSpeeds){
        Pose3d initPose = new Pose3d(swervePose).transformBy(shooterPosition);
        Translation3d initTranslation =  initPose.getTranslation();

        Rotation3d initRotation = initPose.getRotation();

        Vector<N3> initVector = initRotation.getQuaternion().toRotationVector();

        ChassisSpeeds gloabalChassisSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(localChassisSpeeds, swervePose.getRotation());
        double x = initVector.get(0) * launchSpeeds + gloabalChassisSpeeds.vxMetersPerSecond;
        double y = initVector.get(1) * launchSpeeds + gloabalChassisSpeeds.vyMetersPerSecond;
        double z = initVector.get(2) * launchSpeeds;

        addFlyingObject(initTranslation, new Translation3d(x,y,z));
    }

    public void addFlyingObject(Pose2d swervePose, Translation3d shooterPosition, Rotation3d shootRotation, ChassisSpeeds localChassisSpeeds, double launchSpeeds){
        Pose3d initPose = new Pose3d(swervePose).transformBy(new Transform3d(shooterPosition, shootRotation));
        Translation3d initTranslation =  initPose.getTranslation();

        Rotation3d initRotation = initPose.getRotation();

        double ix = Math.cos(initRotation.getZ()) * Math.cos(initRotation.getY());
        double iy = Math.sin(initRotation.getZ()) * Math.cos(initRotation.getY());
        double iz = Math.sin(initRotation.getY());

        ChassisSpeeds gloabalChassisSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(localChassisSpeeds, swervePose.getRotation());
        double x = ix * launchSpeeds + gloabalChassisSpeeds.vxMetersPerSecond;
        double y = iy * launchSpeeds + gloabalChassisSpeeds.vyMetersPerSecond;
        double z = iz * launchSpeeds;

        addFlyingObject(initTranslation, new Translation3d(x,y,z));
    }

    public void periodic(){
        try {
            Translation3d[] arr = new Translation3d[flyingObjects.size()];
        for (int i = 0; i < flyingObjects.size(); i++){
            Pair<Translation3d, Translation3d> pair = flyingObjects.get(i);
            Translation3d initPose = pair.getFirst();
            Translation3d velocity = pair.getSecond();
            Translation3d finalPose = initPose.plus(velocity.times(.02));
            Translation3d finalVelocity = velocity.plus(new Translation3d(0,0,-9.81*.02));

            flyingObjects.set(i, new Pair<Translation3d,Translation3d>(finalPose, finalVelocity));
            arr[i] = flyingObjects.get(i).getFirst();
        }

        flyingObjects.removeIf(o -> (o.getFirst().getZ() < 0));
        flyingObjectsPublisher.accept(arr);
        } catch (Exception e) {
            System.out.println("AHHH2");
        }
        
    }
}
