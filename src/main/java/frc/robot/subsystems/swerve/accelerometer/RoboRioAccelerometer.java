package frc.robot.subsystems.swerve.accelerometer;

import edu.wpi.first.wpilibj.BuiltInAccelerometer;

public class RoboRioAccelerometer implements AccelerometerIO{
    private final BuiltInAccelerometer accelerometer;
    public RoboRioAccelerometer(){
        accelerometer = new BuiltInAccelerometer();
    }

    @Override
    public double getX() {
        return accelerometer.getX();
    }
    @Override
    public double getY() {
        return accelerometer.getY();
    }
    @Override
    public double getZ() {
        return accelerometer.getZ();
    }
}
