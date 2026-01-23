package frc.robot.subsystems.swerve.simSwerve;

import java.util.Arrays;
import java.util.function.Consumer;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SelfControlledSwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.gamepieces.GamePieceOnFieldSimulation;
import org.ironmaple.simulation.seasonspecific.crescendo2024.CrescendoNoteOnField;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeAlgaeOnField;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralAlgaeStack;
import org.ironmaple.utils.FieldMirroringUtils;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveControlParameters;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.GameData;
import frc.robot.subsystems.swerve.SwerveDriveIO;
import frc.robot.util.mapleSim.Bootleg2022;

//TODO Imperfect, better can be done
public class SimSwerve implements SwerveDriveIO {

    private final SelfControlledSwerveDriveSimulation simulatedDrive;

    private final NetworkTable instance = NetworkTableInstance.getDefault().getTable("RealData");

    private final StructPublisher<Pose2d> posePublisher = instance
        .getStructTopic("RealPose", Pose2d.struct).publish();



    private Rotation2d yawOffset = new Rotation2d();
    
    private Thread updateThread = new Thread();

    public SimSwerve() {
        // For your own code, please configure your drivetrain properly according to the documentation
        DriveTrainSimulationConfig config = SimSwerveConstants.driveTrainSimulationConfig;

        // Creating the SelfControlledSwerveDriveSimulation instance
        //I SPENT HOURS TO FIND THIS LINE!!!!!!!!!!!!!!!!!!!!!!!!! AHHHHHHHHHHHHHHHHHHHHHHHHHH.
        this.simulatedDrive = new SelfControlledSwerveDriveSimulation(
                new SwerveDriveSimulation(config, new Pose2d(7, 5, new Rotation2d())), VecBuilder.fill(0.1,0.1,0.1), VecBuilder.fill(0,0,0));

        // Register the drivetrain simulation to the simulation world
        SimulatedArena.overrideInstance(new SimulatedArena(new SimulatedArena.FieldMap() {
            
        }) {

            @Override
            public void placeGamePiecesOnField() {
            //     Translation2d[] bluePositions = new Translation2d[] {
            //         GameData.getCargoPose(1,false).getTranslation()
            //         ,GameData.getCargoPose(2,false).getTranslation()
            //         ,GameData.getCargoPose(3,false).getTranslation()
            //         ,GameData.getCargoPose(4,false).getTranslation()
            //         ,GameData.getCargoPose(5,false).getTranslation()
            //         ,GameData.getCargoPose(6,false).getTranslation()
            //     };
            //     for (Translation2d position : bluePositions) super.addGamePiece(new ReefscapeAlgaeOnField(position));

            //     Translation2d[] redPositions = new Translation2d[] {
            //         GameData.getCargoPose(1,true).getTranslation()
            //         ,GameData.getCargoPose(2,true).getTranslation()
            //         ,GameData.getCargoPose(3,true).getTranslation()
            //         ,GameData.getCargoPose(4,true).getTranslation()
            //         ,GameData.getCargoPose(5,true).getTranslation()
            //         ,GameData.getCargoPose(6,true).getTranslation()
            //     };
            //     for (Translation2d position : redPositions) super.addGamePiece(new CrescendoNoteOnField(position));
            }
        });
        SimulatedArena.getInstance().addDriveTrainSimulation(simulatedDrive.getDriveTrainSimulation());
        SimulatedArena.getInstance().placeGamePiecesOnField();
        updateThread = new Thread(()->{
            while(true){
                try {
                    // SimulatedArena.getInstance().simulationPeriodic();
                    try {
                        simulatedDrive.periodic();
                    } catch (Exception e) {
                        System.out.println("AHHHHH");
                    }
                    posePublisher.accept(simulatedDrive.getActualPoseInSimulationWorld());
                    Thread.sleep(20);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
        });
        updateThread.setDaemon(true);
        updateThread.start();
        Bootleg2022.addMainDrive(simulatedDrive.getDriveTrainSimulation());
    }

    @Override
    public void setControl(SwerveRequest request) {
        //TODO this is probably really slow, if someone finds a better way please fix
        ChassisSpeeds speeds = new ChassisSpeeds();
        request.apply(new SwerveControlParameters(), new SwerveModule[]{});
        boolean fieldRelative = false;
        boolean discretize = true;

        if (request instanceof SwerveRequest.ApplyFieldSpeeds){
            speeds = ((SwerveRequest.ApplyFieldSpeeds)request).Speeds;
            fieldRelative = true;
        }
        if (request instanceof SwerveRequest.ApplyRobotSpeeds){
            speeds = ((SwerveRequest.ApplyRobotSpeeds)request).Speeds;
        }
        if (request instanceof SwerveRequest.FieldCentric){
            Translation2d translation = new Translation2d(((SwerveRequest.FieldCentric)request).VelocityX,((SwerveRequest.FieldCentric)request).VelocityY);

            speeds = new ChassisSpeeds(
                translation.getX() * Math.cos(yawOffset.getRadians()) - translation.getY() * Math.sin(yawOffset.getRadians()),
                translation.getX() * Math.sin(yawOffset.getRadians()) + translation.getY() * Math.cos(yawOffset.getRadians()),
                ((SwerveRequest.FieldCentric)request).RotationalRate);
            fieldRelative = true;
        }
        if (request instanceof SwerveRequest.FieldCentricFacingAngle){
            //TODO
            fieldRelative = true;
        }
        if (request instanceof SwerveRequest.RobotCentric){
            speeds = new ChassisSpeeds(
                ((SwerveRequest.RobotCentric)request).VelocityX,
                ((SwerveRequest.RobotCentric)request).VelocityY,
                ((SwerveRequest.RobotCentric)request).RotationalRate);
        }
        if (request instanceof SwerveRequest.PointWheelsAt){
            Rotation2d direction = ((SwerveRequest.PointWheelsAt)request).ModuleDirection;
            runSwerveStates(
                new SwerveModuleState[]{
                    new SwerveModuleState(0,direction),
                    new SwerveModuleState(0,direction),
                    new SwerveModuleState(0,direction),
                    new SwerveModuleState(0,direction)
            });
            return;
        }
        if (request instanceof SwerveRequest.Idle){
            speeds = new ChassisSpeeds();
        }
        if (request instanceof SwerveRequest.SwerveDriveBrake){
            runSwerveStates(
                new SwerveModuleState[]{
                    new SwerveModuleState(0,Rotation2d.fromDegrees(45)),
                    new SwerveModuleState(0,Rotation2d.fromDegrees(-45)),
                    new SwerveModuleState(0,Rotation2d.fromDegrees(-45)),
                    new SwerveModuleState(0,Rotation2d.fromDegrees(45))
            });
            return;
        }
        runChassisSpeeds(speeds, fieldRelative, discretize);
    }

    private void runChassisSpeeds(ChassisSpeeds speeds, boolean fieldCentric, boolean discretize){
        this.simulatedDrive.runChassisSpeeds(
                speeds,
                new Translation2d(),
                fieldCentric,
                discretize);
    }

    public void runSwerveStates(SwerveModuleState[] targets) {
        this.simulatedDrive.runSwerveStates(targets);
    }

    @Override
    public Pose2d getPose() {
        return simulatedDrive.getOdometryEstimatedPose();
    }

    public Pose2d getRealPose(){
        return simulatedDrive.getActualPoseInSimulationWorld();
    }

    double resetTime = -1;
    @Override
    public void resetPose(Pose2d pose) {
        resetTime = Timer.getFPGATimestamp();
        simulatedDrive.setSimulationWorldPose(pose);
        simulatedDrive.resetOdometry(pose);
        SimulatedArena.getInstance().resetFieldForAuto();
    }

    @Override
    public void seedFieldRelative(Rotation2d rot) {
        yawOffset = rot;
    }

    @Override
    public void addVisionMeasurement(Pose2d pose, double timestep, Vector<N3> stdDev) {
        if (currentTimeToFPGA(timestep) < resetTime){
            return;
        }
        //TODO broken currently
        simulatedDrive.addVisionEstimation(pose, currentTimeToFPGA(timestep), VecBuilder.fill(0,0,0));
    }

    private double currentTimeToFPGA(double currentTime) {
        return (Timer.getFPGATimestamp() - Utils.getCurrentTimeSeconds()) + currentTime;
    }

    private Thread telemetryThread = new Thread();
    private Consumer<SwerveDriveState> consumer = null;
    @Override
    public void registerTelemetry(Consumer<SwerveDriveState> consumer) {
        if (this.consumer != consumer){
            // telemetryThread.interrupt();
            telemetryThread = new Thread(()->{
                while(true){
                    try {
                        consumer.accept(getState());
                        Thread.sleep(20);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                }
                

            });
            telemetryThread.setDaemon(true);
            telemetryThread.start();
        }
    }

    @Override
    public Rotation2d getYaw() {
        return simulatedDrive.getOdometryEstimatedPose().getRotation();
    }

    @Override
    public Rotation2d getYawOffset() {
        return yawOffset;
    }

    @Override
    public ChassisSpeeds getSpeeds() {
        return simulatedDrive.getMeasuredSpeedsRobotRelative(true);
    }

    @Override
    public SwerveDriveState getState() {
        SwerveDriveState state = new SwerveDriveState();
        state.Pose = getPose();
        state.ModuleStates = simulatedDrive.getMeasuredStates();
        state.ModuleTargets = simulatedDrive.getSetPointsOptimized();
        state.ModulePositions = simulatedDrive.getLatestModulePositions();
        state.Speeds = getSpeeds();
        
        return state;
    }
}
