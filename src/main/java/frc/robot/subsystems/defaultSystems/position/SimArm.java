package frc.robot.subsystems.defaultSystems.position;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class SimArm implements PositionIO{

    //Set PID vals
    private PIDController pidController;

    private double targetPosition = 0;

    private boolean voltageOut = false;
    private double outputVolts = 0;

    private boolean stopped = false;

    private Thread updateThread;

    private final SingleJointedArmSim singleJointedArmSim;

    //Main Loop for SimArm
    public SimArm(SingleJointedArmSim singleJointedArmSim, PIDController pidController){
        this.singleJointedArmSim = singleJointedArmSim;
        this.pidController = pidController;
        updateThread = new Thread(()->{
            while(true){
                try {
                    if (stopped){
                        singleJointedArmSim.setInputVoltage(0);
                    }else{
                        //Move arm based on target Values
                        if (voltageOut){
                            singleJointedArmSim.setInputVoltage(outputVolts);
                        }else{
                            singleJointedArmSim.setInputVoltage(this.pidController.calculate(getPosition(), targetPosition));
                        }
                    }
                    //Update positions every 20 ms
                    singleJointedArmSim.update(.02);
                    //Sleep 20 ms
                    Thread.sleep(20);

                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
        });
        updateThread.setDaemon(true); 
        updateThread.start();
    }
  
    @Override
    public void setPosition(double position) {
        voltageOut = false;
        targetPosition = position;
        stopped = false;
    }

    @Override
    public void stop() {
        stopped = true;
    }

    @Override
    public double getPosition() {
        return Units.radiansToRotations(singleJointedArmSim.getAngleRads());
    }

    @Override
    public double getTarget() {
        return targetPosition;
    }

    @Override
    public void setVoltage(double volts) {
        voltageOut = true;
        outputVolts = volts;
        stopped = false;
    }
}
