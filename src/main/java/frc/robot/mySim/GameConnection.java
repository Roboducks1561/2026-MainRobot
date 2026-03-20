package frc.robot.mySim;

import java.io.*;
import java.lang.reflect.Type;
import java.net.*;
import java.util.HashMap;

import com.google.gson.Gson;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GameConnection {

    private static GameConnection instance;
    private static Pose2d robotPose;
    private static ServerSocket serverSocket;

    private static Gson sending;
    private static Gson recieved;

    private GameConnection(){
        try {
            serverSocket = new ServerSocket(7716);
        } catch (Exception e) {
            // TODO: handle exception
        }
        Thread comThread = new Thread(()->{
            try {
                while (true) {
                    Socket clientSocket = serverSocket.accept();

                    try (BufferedReader in = new BufferedReader(new InputStreamReader(clientSocket.getInputStream()));
                        PrintWriter out = new PrintWriter(clientSocket.getOutputStream(), true)) {

                        String request = in.readLine();
                        CollisionVector collisionVector = new Gson().fromJson(request, CollisionVector.class);
                        if (m_drivetrain != null && collisionVector.m_xN+collisionVector.m_yN != 0 ){
                            
                        }
                        String response = new Gson().toJson(robotPose);
                        out.println(response);
                    }
                    clientSocket.close();
                    Thread.sleep((long) 20);
                }
            } catch (Exception e) {
                System.out.println("Main thread ended");
            }
        });
        comThread.start();
    }

    public static void initConnection(){
        if (instance == null){
            instance = new GameConnection();
        }
    }
}