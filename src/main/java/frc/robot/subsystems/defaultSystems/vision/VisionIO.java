package frc.robot.subsystems.defaultSystems.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.util.LimelightHelpers;

public interface VisionIO {

    /**
     * used to help megatag 2 find pose more accurately, do periodically
     * @param yaw
     */
    public void setOrientation(Rotation2d yaw);

    public LimelightHelpers.PoseEstimate getPoseEstimate();

    public String getName();
}
