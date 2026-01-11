package frc.robot.subsystems.swerve.accelerometer;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.Utils;

public class SimAccelerometer implements AccelerometerIO{

    private Thread updateThread = new Thread();

    private double lastxVel = 0;
    private double lastyVel = 0;
    private double lastTimeStamp = 0;

    private double xAccel = 0;
    private double yAccel = 0;

    /**
     * All returned values are in G forces
     * @param xVel robot relative
     * @param yVel robot relative
     */
    public SimAccelerometer(DoubleSupplier xVel, DoubleSupplier yVel){
        lastxVel = xVel.getAsDouble();
        lastyVel = yVel.getAsDouble();
        lastTimeStamp = Utils.getCurrentTimeSeconds();

        updateThread = new Thread(()->{
            while(true){
                try {
                    Thread.sleep(20);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                xAccel = (xVel.getAsDouble()-lastxVel)/(Utils.getCurrentTimeSeconds() - lastTimeStamp);
                yAccel = (yVel.getAsDouble()-lastyVel)/(Utils.getCurrentTimeSeconds() - lastTimeStamp);
                lastTimeStamp = Utils.getCurrentTimeSeconds();
                lastxVel = xVel.getAsDouble();
                lastyVel = yVel.getAsDouble();
            }
        });
        updateThread.setDaemon(true);
        updateThread.start();
    }

    @Override
    public double getX() {
        return xAccel/9.81;
    }

    @Override
    public double getY() {
        return yAccel/9.81;
    }

    @Override
    public double getZ() {
        return -1;
    }
}
