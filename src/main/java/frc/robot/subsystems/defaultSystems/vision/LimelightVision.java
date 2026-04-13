package frc.robot.subsystems.defaultSystems.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightHelpers.PoseEstimate;

public class LimelightVision implements VisionIO{

    private final String name;
    public LimelightVision(String name){
        this.name = name;
    }

    @Override
    public void setOrientation(Rotation2d yaw) {
        LimelightHelpers.SetRobotOrientation(name, yaw.getDegrees(), 0, 0, 0, 0, 0);
    }

    @Override
    public PoseEstimate getPoseEstimate() {
        // return (DriverStation.isAutonomous() ? LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name) : LimelightHelpers.getBotPoseEstimate_wpiBlue(name));
        return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
        // return LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
    }

    @Override
    public String getName() {
        return name;
    }
    
}
