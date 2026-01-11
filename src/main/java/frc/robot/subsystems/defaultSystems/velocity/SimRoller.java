package frc.robot.subsystems.defaultSystems.velocity;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class SimRoller implements VelocityIO{

    //Set what type of gear and how many gears to use
    
    //Set PID values
    private PIDController pidController;

    private double targetVelocity = 0;

    private boolean stopped = false;

    private boolean voltageOut = false;
    private double outputVolts = 0;

    private Thread updateThread;


    private final FlywheelSim rollerSim;

    public SimRoller(FlywheelSim rollerSim, PIDController pidController){
        this.rollerSim = rollerSim;
        this.pidController = pidController;
        updateThread = new Thread(()->{
            while(true){
                try {
                    if (stopped){
                        //Stop roller
                        rollerSim.setInputVoltage(0);
                    }else{
                        //Run roller with set values
                        if (voltageOut){
                            rollerSim.setInputVoltage(outputVolts);
                        }else{
                            rollerSim.setInputVoltage(this.pidController.calculate(getVelocity(), targetVelocity));
                        }
                    }
                    //Update every 20 ms
                    rollerSim.update(.02);
                    //Sleep every 20 ms
                    Thread.sleep(20);
                } catch (InterruptedException e) {
                    //Print StackTrace if exception is caught
                    e.printStackTrace();
                }
            }
        });
        updateThread.setDaemon(true); 
        updateThread.start();
    }
  
    @Override
    public void setVelocity(double rps) {
        targetVelocity = rps;
        stopped = false;
        voltageOut = false;
    }

    @Override
    public void stop() {
        stopped = true;
    }

    /**
     * in rotations of mechanism per second
     */
    @Override
    public double getVelocity() {
        return (Units.radiansToRotations(rollerSim.getAngularVelocityRadPerSec()));
    }

    @Override
    public double getTarget() {
        return targetVelocity;
    }

    @Override
    public double getCurrent() {
        return rollerSim.getCurrentDrawAmps();
    }

    @Override
    public double getAcceleration() {
        return (Units.radiansToRotations(rollerSim.getAngularAccelerationRadPerSecSq()));
    }

    @Override
    public void setVoltage(double volts) {
        voltageOut = true;
        outputVolts = volts;
        stopped = false;
    }
}
