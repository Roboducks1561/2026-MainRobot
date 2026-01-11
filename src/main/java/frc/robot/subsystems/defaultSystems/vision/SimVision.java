/*
 * MIT License
 *
 * Copyright (c) PhotonVision
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

 package frc.robot.subsystems.defaultSystems.vision;
 
 import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
 import edu.wpi.first.math.geometry.Pose2d;
 import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constants.LimelightConstants;
import frc.robot.subsystems.swerve.SwerveDriveIO;
import frc.robot.subsystems.swerve.simSwerve.SimSwerve;
import frc.robot.util.LimelightHelpers.PoseEstimate;
import frc.robot.util.LimelightHelpers.RawFiducial;

 import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
 import org.photonvision.PhotonCamera;
 import org.photonvision.PhotonPoseEstimator;
 import org.photonvision.PhotonPoseEstimator.PoseStrategy;
 import org.photonvision.simulation.PhotonCameraSim;
 import org.photonvision.simulation.SimCameraProperties;
 import org.photonvision.simulation.VisionSystemSim;
 import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.Utils;
 
//TODO imperfect, but sim, so if it works, doesn't matter so much
 public class SimVision extends SubsystemBase implements VisionIO{
     private final PhotonCamera camera;
     private final PhotonPoseEstimator photonEstimator;
     private Matrix<N3, N1> curStdDevs;
 
     // Simulation
     private PhotonCameraSim cameraSim;
     private VisionSystemSim visionSim;

     private final SwerveDriveIO driveIO;
     private boolean completeSim = false;

     private final String name;
 
     private Thread updateThread = new Thread();

     public SimVision(SwerveDriveIO drivetrain, String name, Transform3d transform) {
        this.name = name;
        this.driveIO = drivetrain;
        if (driveIO instanceof SimSwerve){
            completeSim = true;
        }

        camera = new PhotonCamera(name);
        Transform3d last = transform;
        Transform3d newTransform = new Transform3d(last.getX(), last.getY(), last.getZ(),
             new Rotation3d(last.getRotation().getX(),-last.getRotation().getY(),last.getRotation().getZ()));
        photonEstimator =
                new PhotonPoseEstimator(LimelightConstants.K_TAG_LAYOUT, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, newTransform);
        photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        // ----- Simulation
        if (Robot.isSimulation()) {
            // Create the vision system simulation which handles cameras and targets on the field.
            visionSim = new VisionSystemSim(name);
            // Add all the AprilTags inside the tag layout as visible targets to this simulated field.
            visionSim.addAprilTags(LimelightConstants.K_TAG_LAYOUT);
            // Create simulated camera properties. These can be set to mimic your actual camera.
            var cameraProp = new SimCameraProperties();
            cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(90));
            cameraProp.setCalibError(0,0);//0.35, 0.10);
            cameraProp.setFPS(15);
            cameraProp.setAvgLatencyMs(50);//50);
            cameraProp.setLatencyStdDevMs(15);//15);
            // Create a PhotonCameraSim which will update the linked PhotonCamera's values with visible
            // targets.
            // cameraSim = new PhotonCameraSim(camera, cameraProp);
            cameraSim = new PhotonCameraSim(camera, cameraProp, LimelightConstants.K_TAG_LAYOUT);
            // Add the simulated camera to view the targets on this simulated field.
            visionSim.addCamera(cameraSim, newTransform);

            cameraSim.enableDrawWireframe(true);

            updateThread = new Thread(()->{
                while(true){
                    double startTime = Utils.getCurrentTimeSeconds();
                    try {
                        if (completeSim){
                            visionSim.update(((SimSwerve)driveIO).getRealPose());
                            Thread.sleep(20);
                            continue;
                        }
                        visionSim.update(driveIO.getPose());
                        Thread.sleep(20);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                    if (Utils.getCurrentTimeSeconds() - startTime > .02){
                        System.out.println("Vision thread exceeded limit: "+(Utils.getCurrentTimeSeconds() - startTime));
                    }
                }
            });
            updateThread.setName("Vision Update Thread");
            updateThread.setDaemon(true);
            updateThread.start();
        }
     }
 
     /**
      * The latest estimated robot pose on the field from vision data. This may be empty. This should
      * only be called once per loop.
      *
      * <p>Also includes updates for the standard deviations, which can (optionally) be retrieved with
      * {@link getEstimationStdDevs}
      *
      * @return An {@link EstimatedRobotPose} with an estimated pose, estimate timestamp, and targets
      *     used for estimation.
      */
     public PoseEstimate getPoseEstimate() {
        
         Optional<EstimatedRobotPose> visionEst = Optional.empty();
         for (var change : camera.getAllUnreadResults()) {
             visionEst = photonEstimator.update(change);
         }
         
         if (visionEst.isEmpty()){
            return new PoseEstimate();
         }
         int size = visionEst.get().targetsUsed.size();
         RawFiducial[] fiducials = new RawFiducial[size];
         int i = 0;
         double totalDist = 0;
         double totalArea = 0;
         double delay = .05;
         for (PhotonTrackedTarget target : visionEst.get().targetsUsed){
            RawFiducial fiducial = new RawFiducial(target.fiducialId,
             target.yaw,
              target.pitch,
               target.area,
                target.bestCameraToTarget.getTranslation().toTranslation2d().getDistance(new Translation2d(0,0)),
                target.bestCameraToTarget.getTranslation().toTranslation2d().getDistance(new Translation2d(0,0)),
                 target.poseAmbiguity);
            fiducials[i] = fiducial;
            totalArea+=fiducial.ta;
            totalDist+=fiducial.distToRobot;
            i++;
         }
         PoseEstimate estimate = new PoseEstimate(
            visionEst.get().estimatedPose.toPose2d(),
            Utils.fpgaToCurrentTime(visionEst.get().timestampSeconds),
            delay,
            size,
            0,
            totalDist/size,
            totalArea/size,
            fiducials,
            true);

         return estimate;
     }
 
     /**
      * Returns the latest standard deviations of the estimated pose from {@link
      * #getEstimatedGlobalPose()}, for use with {@link
      * edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}. This should
      * only be used when there are targets visible.
      */
     public Matrix<N3, N1> getEstimationStdDevs() {
         return curStdDevs;
     }
 
     /** Reset pose history of the robot in the vision system simulation. */
     public void resetSimPose(Pose2d pose) {
         if (Robot.isSimulation()) visionSim.resetRobotPose(pose);
     }


    /**
     * Unused here
     */
    @Override
    public void setOrientation(Rotation2d yaw) {
        
    }

    @Override
    public void periodic() {
        
    }

    public String getName(){
        return name;
    }
 }