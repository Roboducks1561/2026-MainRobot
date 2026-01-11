// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.GameData;

//TODO, math could be improved likely
public class PoseEX {

    public static Rotation2d getPoseAngle(Pose2d mainPose, Pose2d comparingPose) {
        double deltaX = comparingPose.getX() - mainPose.getX();
        double deltaY = comparingPose.getY() - mainPose.getY();
        return Rotation2d.fromRadians(Math.atan2(deltaY, deltaX));
    }

    public static Pose2d mirrorPose(Pose2d initPose){
        return new Pose2d(GameData.fieldSizeX-initPose.getX(),initPose.getY(),Rotation2d.fromDegrees(180-initPose.getRotation().getDegrees()));
    }

    public static Pose2d pose180(Pose2d initPose){
        double newDegrees = initPose.getRotation().getDegrees() - 180;
        if (newDegrees < -180){
            newDegrees+=360;
        }
        return new Pose2d(GameData.fieldSizeX-initPose.getX(), GameData.fieldSizeY-initPose.getY(), Rotation2d.fromDegrees(newDegrees));
    }
    
    public static double getDistanceFromPoseMeters(Pose2d mainPose, Pose2d comparingPose) {
        return Math.sqrt(Math.pow(comparingPose.getX()-mainPose.getX(),2)+Math.pow(comparingPose.getY()-mainPose.getY(),2));
    }
    
    public static Rotation2d getYawFromPose(Pose2d mainPose, Pose2d comparingPose) {
        double deltaX = comparingPose.getX() - mainPose.getX();
        double deltaY = comparingPose.getY() - mainPose.getY();

        double angleRadians = ((Math.atan(deltaY/deltaX)));

        // Convert the angle to Rotation2d
        Rotation2d rotation = Rotation2d.fromRadians(angleRadians - mainPose.getRotation().getRadians());
        if (mainPose.getX()>comparingPose.getX()){
            if (rotation.getDegrees()>0){
                rotation = Rotation2d.fromDegrees(Units.radiansToDegrees(angleRadians) - mainPose.getRotation().getDegrees()-180);
            }else{
                rotation = Rotation2d.fromDegrees(Units.radiansToDegrees(angleRadians) - mainPose.getRotation().getDegrees()+180);
            }
        }
        return rotation;
    }

    public static Rotation2d getPitchFromPose(Pose3d mainPose, Pose3d comparingPose) {
        double deltaDist;
        double deltaZ;
        deltaDist = Math.sqrt(Math.pow(comparingPose.getX() - mainPose.getX(),2)+Math.pow(comparingPose.getY() - mainPose.getY(),2));
        
        deltaZ = comparingPose.getZ() - mainPose.getZ();
        
        double angleZ = ((Math.atan(deltaZ/deltaDist)));
        return Rotation2d.fromRadians(angleZ - mainPose.getRotation().getY());
    }

    public static Pose2d getInbetweenPose2d(Pose2d mainPose, Pose2d comparingPose, double distFrom){
        return getInbetweenPose2d(mainPose, comparingPose, new Rotation2d(), distFrom);
    }

    public static Pose2d getInbetweenPose2d(Pose2d mainPose, Pose2d comparingPose, Rotation2d rotation, double distFrom){

        double rise = comparingPose.getY()-mainPose.getY();
        double run = comparingPose.getX()-mainPose.getX();
        
        double amount = Math.sqrt((rise*rise)+(run*run));
        double newRise = rise*(distFrom/amount);
        double newRun = run*(distFrom/amount);

        double newX = comparingPose.getX() - newRun;
        double newY = comparingPose.getY() - newRise;
        Pose2d returnPose = new Pose2d(newX, newY, rotation);
        return returnPose;
    }

    public static Pose2d getInbetweenPose2d(Pose2d mainPose, Pose2d comparingPose){
        return new Pose2d((mainPose.getX() + comparingPose.getX())/2, (mainPose.getY() + comparingPose.getY())/2, new Rotation2d());
    }

    public static Pose2d rotatePose(Pose2d pose, Rotation2d rotation){
        Pose2d newPose = new Pose2d(pose.getX(), pose.getY(), correctedRotation(Rotation2d.fromDegrees(pose.getRotation().getDegrees() + rotation.getDegrees())));
        return newPose;
    }

    public static Pose2d linearPlus(Pose2d pose, Pose2d add){
        Pose2d newPose = new Pose2d(pose.getX() + add.getX(), pose.getY() + add.getY(), correctedRotation(Rotation2d.fromDegrees(pose.getRotation().getDegrees() + add.getRotation().getDegrees())));
        return newPose;
    }

    public static Rotation2d correctedRotation(Rotation2d rot){
        double degrees = rot.getDegrees();
        degrees = degrees%360;
        degrees = degrees > 180 ? degrees-360 : degrees < -180 ? degrees+360 : degrees;
        return Rotation2d.fromDegrees(degrees);
    }

    public static double correctedRotation(double rotations){
        rotations = rotations%1;
        rotations = rotations > .5 ? rotations-1 : rotations < -.5 ? rotations+1 : rotations;
        return rotations;
    }

    public static Pose2d closestTo(Pose2d to, Pose2d... options){
        if (options.length == 0){
            return to;
        }
        double minDist = Double.MAX_VALUE;
        Pose2d closest = new Pose2d();
        for (Pose2d option : options){
            double curDist = to.minus(option).getTranslation().getNorm();
            if (curDist < minDist){
                minDist = curDist;
                closest = option;
            }
        }
        return closest;
    }
}
