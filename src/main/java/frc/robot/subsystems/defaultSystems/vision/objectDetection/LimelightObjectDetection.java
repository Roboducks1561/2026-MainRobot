package frc.robot.subsystems.defaultSystems.vision.objectDetection;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.util.LimelightHelpers;

public class LimelightObjectDetection implements ObjectDetectionIO{

    private final String limelightName;
    public LimelightObjectDetection(String name){
        limelightName = name;
    }

    @Override
    public Rotation2d getHorizontalRotationFromTarget() {
        NetworkTable table = NetworkTableInstance.getDefault().getTable(limelightName);
        NetworkTableEntry tx = table.getEntry("tx");
        return Rotation2d.fromDegrees(tx.getDouble(0.0));
    }

    @Override
    public Rotation2d getVerticalRotationFromPiece() {
        NetworkTable table = NetworkTableInstance.getDefault().getTable(limelightName);
        NetworkTableEntry ty = table.getEntry("ty");
        return Rotation2d.fromDegrees(ty.getDouble(0.0));
    }

    @Override
    public boolean isPiecePresent() {
        if (LimelightHelpers.getTA(limelightName) != 0){
            return true;
        }
        return false;
    }
    
}
