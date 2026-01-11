package frc.robot.constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
// import frc.robot.util.MultiLinearInterpolator;
import frc.robot.util.MultiLinearInterpolator;

public class LimelightConstants {
    public static final String BACKWARD_LIMELIGHT_NAME = "limelight-objdet";
    public static final double MAX_XY_ERROR = 1.0;
    public static final Transform3d BACKWARD_LIMELIGHT_CAMERA_TRANSFORM =
        new Transform3d(new Translation3d(-0.297,0, 0.25146+.0158), new Rotation3d(0, Units.degreesToRadians(2.5), Units.degreesToRadians(-180)));
    public static final String FR_LIMELIGHT_NAME = "limelight-rfront";
    public static final Transform3d FR_LIMELIGHT_CAMERA_TRANSFORM =
        new Transform3d(new Translation3d(0.297,0.297, 0.25146+.0158), new Rotation3d(0, Units.degreesToRadians(10), Units.degreesToRadians(30)));

    public static final String FL_LIMELIGHT_NAME = "limelight-lfront";
    public static final Transform3d FL_LIMELIGHT_CAMERA_TRANSFORM =
        new Transform3d(new Translation3d(-0.297,0.297, 0.25146+.0158), new Rotation3d(180, Units.degreesToRadians(10), Units.degreesToRadians(-30)));
    // public static final String AMP_CAM = "limelight-object";
    // public static final double MAX_XY_ERROR_AMP_CAM = 1.0;
    // public static final Transform3d AMP_CAM_TRANSFORM =
    //     new Transform3d(new Translation3d(0,-.101, .522), new Rotation3d(0, Units.degreesToRadians(-20), Units.degreesToRadians(0)));

    public static final AprilTagFieldLayout K_TAG_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
    public static final double[][] ONE_APRIL_TAG_LOOKUP_TABLE = {
      // {distance in meters, x std deviation, y std deviation, r (in degrees//actually maybe radians) std deviation}
      {0, 0.0, 0.0, 0},
      {1.5, 0.0, 0.0, 0},
      {3, 2, 2, 2},
      {4.5, 4, 4, 4},
      {6, 8, 8, 8},
      {7, 999, 999, 999}
    };
    
    public static final MultiLinearInterpolator ONE_APRIL_TAG_LINEAR_INTERPOLATOR = new MultiLinearInterpolator(ONE_APRIL_TAG_LOOKUP_TABLE);

    public static final double[][] ROTATION_LOOKUP_TABLE = {
      // {distance in meters, x std deviation, y std deviation, r (in degrees) std deviation}
      {0,.1},
      {1.5, .25},
      {3,.5},
      {4.5, 3},
      {6, 1000}
    };
    
    public static final MultiLinearInterpolator ROTATION_LINEAR_INTERPOLATOR = new MultiLinearInterpolator(ROTATION_LOOKUP_TABLE);

    public static final double[][] AUTO_LOOKUP_TABLE = {
      // {distance in meters, x std deviation, y std deviation, r (in degrees//actually maybe radians) std deviation}
      {0, .2, .2, 999999},
      {1.5, .3, .3, 999999},
      {3, .4, .4, 999999},
      {4.5, 1, 1, 999999},
      {6, 2, 2, 999999}
    };

    public static final MultiLinearInterpolator AUTO_INTERPOLATOR = new MultiLinearInterpolator(AUTO_LOOKUP_TABLE);

}
