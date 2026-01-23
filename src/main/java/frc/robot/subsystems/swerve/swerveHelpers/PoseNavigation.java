package frc.robot.subsystems.swerve.swerveHelpers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.util.PoseEX;
import frc.robot.util.Vector2;

public class PoseNavigation {
    private final PIDController speedsPID;
    private final PIDController rotationPID;

    public PoseNavigation(PIDController speedsPID, PIDController rotationPID) {
        this.speedsPID = speedsPID;
        this.rotationPID = rotationPID;
    }

    /**
     * direction to the defined pose with speed constraints
     * @param pose
     * @param target
     * @param speedCap
     * @param radianPSCap
     * @param minDist
     * @return
     */
    public ChassisSpeeds calculateTowardRotation(Rotation2d target, Pose2d current, double radianPSCap) {
        double finalRotation = Math.min(Math.max(rotationPID.calculate(distToCorrectedPoint(target.getRadians(), current.getRotation().getRadians()),0),-radianPSCap),radianPSCap);
        return new ChassisSpeeds(0, 0, finalRotation);
    }
    
    public ChassisSpeeds calculateTowardRotation(Pose2d target, Pose2d current, double radianPSCap) {
        double targetRotation = -PoseEX.getYawFromPose(current, target).getRadians();
        double finalRotation = Math.min(Math.max(rotationPID.calculate(targetRotation,0),-radianPSCap),radianPSCap);
        return new ChassisSpeeds(0, 0, finalRotation);
    }

    /**
     * direction to the defined pose with speed constraints
     * @param pose
     * @param target
     * @param speedCap
     * @param radianPSCap
     * @param minDist
     * @return
     */
    public ChassisSpeeds calculateTowardPose(Pose2d target, Pose2d current, double speedCap, double radianPSCap) {
        Vector2 vector = new Vector2(target.getX()-current.getX(), target.getY()-current.getY());

        Vector2 finalSpeeds = new Vector2(-speedsPID.calculate(vector.x,0),-speedsPID.calculate(vector.y,0));
        if (finalSpeeds.getMagnitude() > speedCap){
            finalSpeeds = finalSpeeds.normalize().multiply(speedCap);
        }

        double finalRotation = Math.min(Math.max(rotationPID.calculate(distToCorrectedPoint(target.getRotation().getRadians(), current.getRotation().getRadians()),0),-radianPSCap),radianPSCap);

        return new ChassisSpeeds(finalSpeeds.x, finalSpeeds.y, finalRotation);
    }

    /**
     * direction to the defined arc with speed constraints
    **/
    public ChassisSpeeds calculateTowardAndPointArc(Pose2d target, Pose2d pointTo, Pose2d current, double distFromCenter, double speedCap, double radianPSCap) {
        Vector2 vector = new Vector2(target.getX()-current.getX(), target.getY()-current.getY());
        Vector2 weights = vector.normalize();

        Vector2 finalSpeeds = new Vector2(-speedsPID.calculate(vector.x,weights.x*distFromCenter),-speedsPID.calculate(vector.y,weights.y*distFromCenter));
        if (finalSpeeds.getMagnitude() > speedCap){
            finalSpeeds = finalSpeeds.normalize().multiply(speedCap);
        }
        double targetRotation = -PoseEX.getYawFromPose(current, pointTo).getRadians();

        double finalRotation = Math.min(Math.max(rotationPID.calculate(targetRotation,0),-radianPSCap),radianPSCap);

        return new ChassisSpeeds(finalSpeeds.x, finalSpeeds.y, finalRotation);
    }

    /**
     * direction to the defined pose with speed constraints
     * @param pose
     * @param target
     * @param speedCap
     * @param radianPSCap
     * @param minDist
     * @return
     */
    public ChassisSpeeds calculateTowardAndPoint(Pose2d target, Pose2d pointTo, Pose2d current, double speedCap, double radianPSCap) {
        Vector2 vector = new Vector2(target.getX()-current.getX(), target.getY()-current.getY());

        Vector2 finalSpeeds = new Vector2(-speedsPID.calculate(vector.x,0),-speedsPID.calculate(vector.y,0));
        if (finalSpeeds.getMagnitude() > speedCap){
            finalSpeeds = finalSpeeds.normalize().multiply(speedCap);
        }
        double targetRotation = -PoseEX.getYawFromPose(current, pointTo).getRadians();

        double finalRotation = Math.min(Math.max(rotationPID.calculate(targetRotation,0),-radianPSCap),radianPSCap);

        return new ChassisSpeeds(finalSpeeds.x, finalSpeeds.y, finalRotation);
    }

    private double distToCorrectedPoint(double pos, double goal) {
        if (Math.abs(goal - pos) < Math.PI) {
            return goal - pos;
        }
        if (pos < 0) {
            return goal - Math.PI * 2 - pos;
        } else {
            return goal + Math.PI * 2 - pos;
        }
    }
}
