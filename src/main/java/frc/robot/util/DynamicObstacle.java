// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.util.ArrayList;
import java.util.List;

import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;

import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;

public class DynamicObstacle {
    /**
     * load dynamic obstacles from a preset navgrid which should be in the pathplanner folder
     * in such as "pathplanner/navgridName.json"
     * @param navgridName
     */
    public static void setDynamicObstacles(String navgridName, Translation2d currentRobotPose){
        Pathfinding.ensureInitialized();
        List<Pair<Translation2d,Translation2d>> dynamicObstacles = new ArrayList<>();
        File navGridFile = new File(Filesystem.getDeployDirectory(), "pathplanner/"+navgridName+".json");

        if (navGridFile.exists()) {
            try (BufferedReader br = new BufferedReader(new FileReader(navGridFile))) {
                StringBuilder fileContentBuilder = new StringBuilder();
                String line;
                while ((line = br.readLine()) != null) {
                    fileContentBuilder.append(line);
                }

                
                String fileContent = fileContentBuilder.toString();
                JSONObject json = (JSONObject) new JSONParser().parse(fileContent);

                double nodeSize = ((Number) json.get("nodeSizeMeters")).doubleValue();
                JSONArray grid = (JSONArray) json.get("grid");
                for (int row = 0; row < grid.size(); row++) {
                    JSONArray rowArray = (JSONArray) grid.get(row);
                    for (int col = 0; col < rowArray.size(); col++) {
                        boolean isObstacle = (boolean) rowArray.get(col);
                        if (isObstacle) {
                            dynamicObstacles.add(new Pair<Translation2d,Translation2d>(new Translation2d(nodeSize*col,nodeSize*row), new Translation2d(nodeSize*col+nodeSize,nodeSize*row+nodeSize)));
                        }
                    }
                }
                Pathfinding.setDynamicObstacles(dynamicObstacles, currentRobotPose);
            } catch (Exception e) {
                // Do nothing, use defaults
            }
        }
    }
    public static void clearDynamicObstacles(Translation2d currentRobotPose){
        Pathfinding.ensureInitialized();
        Pathfinding.setDynamicObstacles(List.of(),currentRobotPose);
    }
}
