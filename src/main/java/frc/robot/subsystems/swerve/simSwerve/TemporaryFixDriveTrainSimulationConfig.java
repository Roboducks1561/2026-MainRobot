// package frc.robot.subsystems.swerve.simSwerve;

// import java.util.function.Supplier;

// import org.ironmaple.simulation.drivesims.GyroSimulation;
// import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
// import org.ironmaple.simulation.drivesims.configs.BoundingCheck;
// import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;

// import edu.wpi.first.units.measure.Distance;
// import edu.wpi.first.units.measure.Mass;

// import static edu.wpi.first.units.Units.Kilograms;
// import static edu.wpi.first.units.Units.Meters;

// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.system.plant.DCMotor;
// import edu.wpi.first.units.measure.Distance;
// import edu.wpi.first.units.measure.Mass;
// import java.util.Arrays;
// import java.util.OptionalDouble;
// import java.util.function.Supplier;
// import org.ironmaple.simulation.drivesims.COTS;
// import org.ironmaple.simulation.drivesims.GyroSimulation;
// import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;

// /**
//  *
//  *
//  * <h1>Stores the configurations for a swerve drive simulation.</h1>
//  *
//  * <p>This class is used to hold all the parameters necessary for simulating a swerve drivetrain, allowing for realistic
//  * performance testing and evaluation.
//  */
// public class TemporaryFixDriveTrainSimulationConfig extends DriveTrainSimulationConfig{
//     public Mass robotMass;
//     public Distance bumperLengthX, bumperWidthY;
//     public Supplier<SwerveModuleSimulation>[] swerveModuleSimulationFactory;
//     public Supplier<GyroSimulation> gyroSimulationFactory;
//     public Translation2d[] moduleTranslations;

//     /**
//      *
//      *
//      * <h2>Ordinary Constructor</h2>
//      *
//      * <p>Creates an instance of {@link TemporaryFixDriveTrainSimulationConfig} with specified parameters.
//      *
//      * @param robotMass the mass of the robot, including bumpers.
//      * @param bumperLengthX the length of the bumper (distance from front to back).
//      * @param bumperWidthY the width of the bumper (distance from left to right).
//      * @param trackLengthX the distance between the front and rear wheels.
//      * @param trackWidthY the distance between the left and right wheels.
//      * @param swerveModuleSimulationFactory the factory that creates appropriate swerve module simulation for the
//      *     drivetrain. You can specify one factory to apply the same configuration over all modules or specify four
//      *     factories in the order (FL, FR, BL, BR).
//      * @param gyroSimulationFactory the factory that creates appropriate gyro simulation for the drivetrain.
//      */
//     public TemporaryFixDriveTrainSimulationConfig(
//             Mass robotMass,
//             Distance bumperLengthX,
//             Distance bumperWidthY,
//             Distance trackLengthX,
//             Distance trackWidthY,
//             Supplier<GyroSimulation> gyroSimulationFactory,
//             Supplier<SwerveModuleSimulation>... swerveModuleSimulationFactory) {
//         super(robotMass, bumperLengthX, bumperWidthY, trackLengthX, trackWidthY, gyroSimulationFactory, swerveModuleSimulationFactory);
//     }

//     @Override
//     public TemporaryFixDriveTrainSimulationConfig withSwerveModule(Supplier<SwerveModuleSimulation> swerveModuleSimulationFactory) {
//         this.swerveModuleSimulationFactory = new Supplier[4];
//         Arrays.fill(this.swerveModuleSimulationFactory, swerveModuleSimulationFactory);
//         return this;
//     }
// }
