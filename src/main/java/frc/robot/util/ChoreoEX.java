// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/** Add your docs here. */
public class ChoreoEX {
    /**
     * needed if you want to preset the starting pose, else, will run without preset even if set
     * @param drivebase
     */

    public static Command getChoreoPath(boolean presetStart, String pathName){
        PathPlannerPath choreoTraj;
        
        try {
            choreoTraj = PathPlannerPath.fromChoreoTrajectory(pathName);
        } catch (Exception e) {
            System.out.println("\n"+"Choreo path of the name: "+pathName+".traj"+" does not exist in deploy/choreo"+"\n");
            return Commands.none();
        }
        if (presetStart){
            PathPoint point = choreoTraj.getPoint(0);
            Pose2d startPose = new Pose2d(point.position.getX(), point.position.getY(), point.rotationTarget.rotation());
            return AutoBuilder.resetOdom(startPose).andThen(AutoBuilder.followPath(choreoTraj));
        }else{
            return AutoBuilder.followPath(choreoTraj);
        }
        
        // ChoreoTrajectory traj = Choreo.getTrajectory(pathName); // 

        //
        //  return Choreo.choreoSwerveCommand(
        //     traj, // 
        //     this::getPose, // 
        //     new PIDController(5, 0.0, 0.0), //
        //     new PIDController(5, 0.0, 0.0), //
        //     new PIDController(5, 0.0, 0.0), //
        //     (ChassisSpeeds speeds) -> //
        //         this.setControl(autoRequest.withSpeeds(speeds)),
        //     () -> {
        //         Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        //         boolean mirror = alliance.isPresent() && alliance.get() == Alliance.Red;
        //         return mirror;
        //     }, // 
        //     this // 
        // );
    }
    /**
     * gives choreoPath with no prestart
     * @param name
     * @return
     */
    public static Command getChoreoPath(String name){
        return getChoreoPath(false, name);
    }

    /**
     * will not run if any names are incorrect or not supplied
     * @param presetStart
     * @param names
     * @return
     */
    public static Command getChoreoGroupPath(boolean presetStart, String... names){
        if (names.length < 1){
            System.out.println("No paths present in groupPath");
            return Commands.none();
        }
        if (getChoreoPath(false, names[0]).getName().equals("InstantCommand")){
            return Commands.none();
        }
        SequentialCommandGroup sequence = new SequentialCommandGroup(Commands.none());
        
        sequence.addCommands(getChoreoPath(presetStart, names[0]));
        
        for (int i = 1; i < names.length; i++){
            Command nextCommand = getChoreoPath(false, names[i]);
            if (nextCommand.getName().equals("InstantCommand")){
                return Commands.none();
            }
            sequence.addCommands(nextCommand);
        }
        return sequence;
    }
}
