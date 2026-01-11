// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve.swerveHelpers;

import java.util.HashMap;
import java.util.function.Supplier;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveControlParameters;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest.NativeSwerveRequest;
import com.ctre.phoenix6.swerve.jni.SwerveJNI;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.util.Vector2;

/** Add your docs here. */
@Deprecated
public class ShareDrive implements NativeSwerveRequest {


    public static class CheapChassisSpeeds{
        /**
         * in meters per second
         */
        public final Vector2 speeds;
        /**
         * in radians per second
         */
        public final double rotation;
        /**
         * declaration time, made so we know when to remove things
         */
        public final double declarationTime;

        public CheapChassisSpeeds(Vector2 vec, double rot){
            speeds = vec;
            rotation = rot;
            declarationTime = Timer.getFPGATimestamp();
        }

        public CheapChassisSpeeds(double vx, double vy, double rot){
            speeds = new Vector2(vx, vy);
            rotation = rot;
            declarationTime = Timer.getFPGATimestamp();
        }

        public CheapChassisSpeeds(ChassisSpeeds speed){
            speeds = new Vector2(speed.vxMetersPerSecond, speed.vyMetersPerSecond);
            rotation = speed.omegaRadiansPerSecond;
            declarationTime = Timer.getFPGATimestamp();
        }

        public static CheapChassisSpeeds discretize(
            double vxMetersPerSecond,
            double vyMetersPerSecond,
            double omegaRadiansPerSecond,
            double dtSeconds) {
            // Construct the desired pose after a timestep, relative to the current pose. The desired pose
            // has decoupled translation and rotation.
            var desiredDeltaPose =
                new Pose2d(
                    vxMetersPerSecond * dtSeconds,
                    vyMetersPerSecond * dtSeconds,
                    new Rotation2d(omegaRadiansPerSecond * dtSeconds));

            // Find the chassis translation/rotation deltas in the robot frame that move the robot from its
            // current pose to the desired pose
            var twist = Pose2d.kZero.log(desiredDeltaPose);

            // Turn the chassis translation/rotation deltas into average velocities
            return new CheapChassisSpeeds(twist.dx / dtSeconds, twist.dy / dtSeconds, twist.dtheta / dtSeconds);
        }
    }

    private final Supplier<Rotation2d> poseRotation;
    private final Supplier<Rotation2d> yawOffset;

    /**
     * This is my custom drive, I made it so that we can have many inputs acting without any need for extra work
     * @param drivetrain
     */
    public ShareDrive(Supplier<Rotation2d> poseRotation, Supplier<Rotation2d> yawOffset){
        this.poseRotation = poseRotation;
        this.yawOffset = yawOffset;
    }

    /**
     * The robotCentric chassis speeds to apply to the drivetrain.
     */
    public ChassisSpeeds Speeds = new ChassisSpeeds();

    /**
     * The total speeds from every type of speed.
     */
    public HashMap<String, CheapChassisSpeeds> allSpeeds = new HashMap<>();

    /**
     * The max velocity of the robot to counteract many adding values, default 5 mps
     */

    public double maxLinearVelocity = 5;

    /**
     * The max rotational velocity of the robot to counteract many adding values, default 2.2PI mps
     */

     public double maxRotationalVelocity = 2.2*Math.PI;

    /**
     * Field-centric wheel force feedforwards to apply in the X direction, in
     * newtons. X is defined as forward according to WPILib convention, so this
     * determines the forward forces to apply.
     * <p>
     * These forces should include friction applied to the ground.
     * <p>
     * The order of the forces should match the order of the modules returned from
     * SwerveDrivetrain.
     */
    public double[] WheelForceFeedforwardsX = {};
    /**
     * Field-centric wheel force feedforwards to apply in the Y direction, in
     * newtons. Y is defined as to the left according to WPILib convention, so this
     * determines the forces to apply to the left.
     * <p>
     * These forces should include friction applied to the ground.
     * <p>
     * The order of the forces should match the order of the modules returned from
     * SwerveDrivetrain.
     */
    public double[] WheelForceFeedforwardsY = {};
    /**
     * The center of rotation the robot should rotate around. This is (0,0) by
     * default, which will rotate around the center of the robot.
     */
    public Translation2d CenterOfRotation = new Translation2d();
    /**
     * The type of control request to use for the drive motor.
     */
    public SwerveModule.DriveRequestType DriveRequestType = SwerveModule.DriveRequestType.OpenLoopVoltage;
    /**
     * The type of control request to use for the drive motor.
     */
    public SwerveModule.SteerRequestType SteerRequestType = SwerveModule.SteerRequestType.Position;
    /**
     * Whether to desaturate wheel speeds before applying. For more information, see
     * the documentation of {@link SwerveDriveKinematics#desaturateWheelSpeeds}.
     */
    public boolean DesaturateWheelSpeeds = true;
    /**
     * The perspective to use when determining which direction is forward.
     */
    public ForwardPerspectiveValue ForwardPerspective = ForwardPerspectiveValue.BlueAlliance;

    /**
     * The field centric speeds of this key to apply.
     * Remember, removeSource if done with these.
     * @param vx
     * @param vy
     * @param radians
     * @param key
     * @return
     */
    public ShareDrive addFieldSpeeds(double vx, double vy, double radians, String key) {
        this.allSpeeds.put("fieldR"+key, new CheapChassisSpeeds(vx, vy, radians));
        return this;
    }

    /**
     * The robot centric speeds of this key to apply.
     * Remember, removeSource if done with these.
     * @param vx
     * @param vy
     * @param radians
     * @param key
     * @return
     */
    public ShareDrive addRobotSpeeds(double vx, double vy, double radians, String key) {
        this.allSpeeds.put("robotR"+key, new CheapChassisSpeeds(vx, vy, radians));
        return this;
    }

    /**
     * The field centric facing orientation speeds of this key to apply.
     * Remember, removeSource if done with these.
     * @param vx
     * @param vy
     * @param radians
     * @param key
     * @return
     */
    public ShareDrive addFieldFacingSpeeds(double vx, double vy, double radians, String key) {
        this.allSpeeds.put("fieldF"+key, new CheapChassisSpeeds(vx, vy,radians));
        return this;
    }

    /**
     * Removes a velocity source by its unique key.
     *
     * @param key the unique identifier
     */
    public void removeSource(String key) {
        allSpeeds.remove(key);
        allSpeeds.remove("fieldR"+key);
        allSpeeds.remove("robotR"+key);
        allSpeeds.remove("fieldF"+key);
    }


    //TODO I do not understand these yet, we've never used them before, if it becomes necessary to have them again i'll figure it out
    // /**
    //  * Modifies the WheelForceFeedforwardsX parameter and returns itself.
    //  * <p>
    //  * Field-centric wheel force feedforwards to apply in the X direction, in
    //  * newtons. X is defined as forward according to WPILib convention, so this
    //  * determines the forward forces to apply.
    //  * <p>
    //  * These forces should include friction applied to the ground.
    //  * <p>
    //  * The order of the forces should match the order of the modules returned from
    //  * SwerveDrivetrain.
    //  *
    //  * @param newWheelForceFeedforwardsX Parameter to modify
    //  * @return this object
    //  */
    // public MainDrive withWheelForceFeedforwardsX(double[] newWheelForceFeedforwardsX) {
    //     this.WheelForceFeedforwardsX = newWheelForceFeedforwardsX;
    //     return this;
    // }
    
    // /**
    //  * Modifies the WheelForceFeedforwardsX parameter and returns itself.
    //  * <p>
    //  * Field-centric wheel force feedforwards to apply in the X direction, in
    //  * newtons. X is defined as forward according to WPILib convention, so this
    //  * determines the forward forces to apply.
    //  * <p>
    //  * These forces should include friction applied to the ground.
    //  * <p>
    //  * The order of the forces should match the order of the modules returned from
    //  * SwerveDrivetrain.
    //  *
    //  * @param newWheelForceFeedforwardsX Parameter to modify
    //  * @return this object
    //  */
    // public MainDrive withWheelForceFeedforwardsX(Force[] newWheelForceFeedforwardsX) {
    //     if (this.WheelForceFeedforwardsX.length != newWheelForceFeedforwardsX.length) {
    //         this.WheelForceFeedforwardsX = new double[newWheelForceFeedforwardsX.length];
    //     }
    //     for (int i = 0; i < this.WheelForceFeedforwardsX.length; ++i) {
    //         this.WheelForceFeedforwardsX[i] = newWheelForceFeedforwardsX[i].in(Newtons);
    //     }

    //     return this;
    // }
    
    // /**
    //  * Modifies the WheelForceFeedforwardsY parameter and returns itself.
    //  * <p>
    //  * Field-centric wheel force feedforwards to apply in the Y direction, in
    //  * newtons. Y is defined as to the left according to WPILib convention, so this
    //  * determines the forces to apply to the left.
    //  * <p>
    //  * These forces should include friction applied to the ground.
    //  * <p>
    //  * The order of the forces should match the order of the modules returned from
    //  * SwerveDrivetrain.
    //  *
    //  * @param newWheelForceFeedforwardsY Parameter to modify
    //  * @return this object
    //  */
    // public MainDrive withWheelForceFeedforwardsY(double[] newWheelForceFeedforwardsY) {
    //     this.WheelForceFeedforwardsY = newWheelForceFeedforwardsY;
    //     return this;
    // }
    
    // /**
    //  * Modifies the WheelForceFeedforwardsY parameter and returns itself.
    //  * <p>
    //  * Field-centric wheel force feedforwards to apply in the Y direction, in
    //  * newtons. Y is defined as to the left according to WPILib convention, so this
    //  * determines the forces to apply to the left.
    //  * <p>
    //  * These forces should include friction applied to the ground.
    //  * <p>
    //  * The order of the forces should match the order of the modules returned from
    //  * SwerveDrivetrain.
    //  *
    //  * @param newWheelForceFeedforwardsY Parameter to modify
    //  * @return this object
    //  */
    // public MainDrive withWheelForceFeedforwardsY(Force[] newWheelForceFeedforwardsY) {
    //     if (this.WheelForceFeedforwardsY.length != newWheelForceFeedforwardsY.length) {
    //         this.WheelForceFeedforwardsY = new double[newWheelForceFeedforwardsY.length];
    //     }
    //     for (int i = 0; i < this.WheelForceFeedforwardsY.length; ++i) {
    //         this.WheelForceFeedforwardsY[i] = newWheelForceFeedforwardsY[i].in(Newtons);
    //     }

    //     return this;
    // }
    
    /**
     * Modifies the CenterOfRotation parameter and returns itself.
     * <p>
     * The center of rotation the robot should rotate around. This is (0,0) by
     * default, which will rotate around the center of the robot.
     *
     * @param newCenterOfRotation Parameter to modify
     * @return this object
     */
    public ShareDrive withCenterOfRotation(Translation2d newCenterOfRotation) {
        this.CenterOfRotation = newCenterOfRotation;
        return this;
    }
    
    /**
     * Modifies the DriveRequestType parameter and returns itself.
     * <p>
     * The type of control request to use for the drive motor.
     *
     * @param newDriveRequestType Parameter to modify
     * @return this object
     */
    public ShareDrive withDriveRequestType(SwerveModule.DriveRequestType newDriveRequestType) {
        this.DriveRequestType = newDriveRequestType;
        return this;
    }
    
    /**
     * Modifies the SteerRequestType parameter and returns itself.
     * <p>
     * The type of control request to use for the drive motor.
     *
     * @param newSteerRequestType Parameter to modify
     * @return this object
     */
    public ShareDrive withSteerRequestType(SwerveModule.SteerRequestType newSteerRequestType) {
        this.SteerRequestType = newSteerRequestType;
        return this;
    }
    
    /**
     * Modifies the DesaturateWheelSpeeds parameter and returns itself.
     * <p>
     * Whether to desaturate wheel speeds before applying. For more information, see
     * the documentation of {@link SwerveDriveKinematics#desaturateWheelSpeeds}.
     *
     * @param newDesaturateWheelSpeeds Parameter to modify
     * @return this object
     */
    public ShareDrive withDesaturateWheelSpeeds(boolean newDesaturateWheelSpeeds) {
        this.DesaturateWheelSpeeds = newDesaturateWheelSpeeds;
        return this;
    }

    /**
     * 
     * @param mps meters per second
     * @return
     */
    public ShareDrive withMaxLinearVelocity(double mps){
        this.maxLinearVelocity = mps;
        return this;
    }

    /**
     * @param rps radians per second
     * @return
     */
    public ShareDrive withMaxRotationalVelocity(double rps){
        this.maxRotationalVelocity = rps;
        return this;
    }

    public StatusCode apply(SwerveControlParameters parameters, SwerveModule... modulesToApply) {
        String[] keys = allSpeeds.keySet().toArray(new String[0]);
        CheapChassisSpeeds[] values = allSpeeds.values().toArray(new CheapChassisSpeeds[0]);

        for (int i = 0; i < allSpeeds.size(); i++){
            if (Timer.getFPGATimestamp() - values[i].declarationTime > .05){
                removeSource(keys[i]);
            }
        }

        keys = allSpeeds.keySet().toArray(new String[0]);
        values = allSpeeds.values().toArray(new CheapChassisSpeeds[0]);

        Vector2 finalSpeeds = new Vector2(0, 0);
        double finalRot = 0;

        double drivetrainRotation = poseRotation.get().getRadians();
        
        double drivetrainOffset = yawOffset.get().getRadians();

        
        for (int i = 0; i < allSpeeds.size(); i++){
            Vector2 vec = values[i].speeds;
            
            if (keys[i].substring(0,6).equals("robotR")){
                finalSpeeds.x += vec.x;
                finalSpeeds.y += vec.y;
                finalRot += values[i].rotation;
                continue;
            }
            double time = Timer.getFPGATimestamp();
            CheapChassisSpeeds newSpeeds = CheapChassisSpeeds.discretize(vec.x,vec.y, values[i].rotation, time - lastTime);
            vec = newSpeeds.speeds;
            lastTime = time;
            
            finalRot += newSpeeds.rotation;
            if (keys[i].substring(0,6).equals("fieldF")){
                vec = vec.rotate(-drivetrainRotation+drivetrainOffset);
                finalSpeeds.x += vec.x;
                finalSpeeds.y += vec.y;
                continue;
            }

            if (keys[i].substring(0,6).equals("fieldR")){
                vec = vec.rotate(-drivetrainRotation);
                finalSpeeds.x += vec.x;
                finalSpeeds.y += vec.y;
                continue;
            }
        }
        if (finalSpeeds.getMagnitude() > maxLinearVelocity){
            finalSpeeds = finalSpeeds.normalize().multiply(maxLinearVelocity);
        }
        finalRot = Math.min(Math.max(finalRot, -maxRotationalVelocity),maxRotationalVelocity);
        Speeds = new ChassisSpeeds(finalSpeeds.x, finalSpeeds.y, finalRot);
        return StatusCode.valueOf(SwerveJNI.JNI_Request_Apply_ApplyFieldSpeeds(parameters.drivetrainId,
            finalSpeeds.x,
            finalSpeeds.y,
            finalRot,
            WheelForceFeedforwardsX,
            WheelForceFeedforwardsY,
            CenterOfRotation.getX(),
            CenterOfRotation.getY(),
            DriveRequestType.value,
            SteerRequestType.value,
            DesaturateWheelSpeeds,
            ForwardPerspective.value));
    }

    private double lastTime = Timer.getFPGATimestamp();

    public void applyNative(int id) {
        String[] keys = allSpeeds.keySet().toArray(new String[0]);
        CheapChassisSpeeds[] values = allSpeeds.values().toArray(new CheapChassisSpeeds[0]);

        for (int i = 0; i < allSpeeds.size(); i++){
            if (Timer.getFPGATimestamp() - values[i].declarationTime > .05){
                removeSource(keys[i]);
            }
        }

        keys = allSpeeds.keySet().toArray(new String[0]);
        values = allSpeeds.values().toArray(new CheapChassisSpeeds[0]);

        Vector2 finalSpeeds = new Vector2(0, 0);
        double finalRot = 0;

        double drivetrainRotation = poseRotation.get().getRadians();
        
        double drivetrainOffset = yawOffset.get().getRadians();

        
        for (int i = 0; i < allSpeeds.size(); i++){
            Vector2 vec = values[i].speeds;
            
            if (keys[i].substring(0,6).equals("robotR")){
                finalSpeeds.x += vec.x;
                finalSpeeds.y += vec.y;
                finalRot += values[i].rotation;
                continue;
            }
            double time = Timer.getFPGATimestamp();
            CheapChassisSpeeds newSpeeds = CheapChassisSpeeds.discretize(vec.x,vec.y, values[i].rotation, time - lastTime);
            vec = newSpeeds.speeds;
            lastTime = time;
            
            finalRot += newSpeeds.rotation;
            if (keys[i].substring(0,6).equals("fieldF")){
                vec = vec.rotate(-drivetrainRotation+drivetrainOffset);
                finalSpeeds.x += vec.x;
                finalSpeeds.y += vec.y;
                continue;
            }

            if (keys[i].substring(0,6).equals("fieldR")){
                vec = vec.rotate(-drivetrainRotation);
                finalSpeeds.x += vec.x;
                finalSpeeds.y += vec.y;
                continue;
            }
        }
        if (finalSpeeds.getMagnitude() > maxLinearVelocity){
            finalSpeeds = finalSpeeds.normalize().multiply(maxLinearVelocity);
        }
        finalRot = Math.min(Math.max(finalRot, -maxRotationalVelocity),maxRotationalVelocity);
        Speeds = new ChassisSpeeds(finalSpeeds.x, finalSpeeds.y, finalRot);
        SwerveJNI.JNI_SetControl_ApplyRobotSpeeds(id,
                finalSpeeds.x,
                finalSpeeds.y,
                finalRot,
                WheelForceFeedforwardsX,
                WheelForceFeedforwardsY,
                CenterOfRotation.getX(),
                CenterOfRotation.getY(),
                DriveRequestType.value,
                SteerRequestType.value,
                DesaturateWheelSpeeds);
    }
}
