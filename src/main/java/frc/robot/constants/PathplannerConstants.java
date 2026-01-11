package frc.robot.constants;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.generated.TunerConstants;

public class PathplannerConstants {

    //TODO you will need to update this for the new robot
    /**
     *  I got these by either calculation, pathplanner telling me, or otherwise, *** whenever making a new one, follow the same form
     * */
    public static RobotConfig pathingConfig = new RobotConfig(50, 1.89, new ModuleConfig(Units.inchesToMeters(TunerConstants.kWheelRadiusInches)
        , 5.45, 2.05, DCMotor.getKrakenX60(1).withReduction(6.746031746031747), 55, 1)
        , new Translation2d(Units.inchesToMeters(TunerConstants.driveBaseRadius), Units.inchesToMeters(TunerConstants.driveBaseRadius)), new Translation2d(Units.inchesToMeters(TunerConstants.driveBaseRadius), Units.inchesToMeters(-TunerConstants.driveBaseRadius)),
         new Translation2d(Units.inchesToMeters(-TunerConstants.driveBaseRadius), Units.inchesToMeters(TunerConstants.driveBaseRadius)), new Translation2d(Units.inchesToMeters(-TunerConstants.driveBaseRadius),Units.inchesToMeters(-TunerConstants.driveBaseRadius)));
}
