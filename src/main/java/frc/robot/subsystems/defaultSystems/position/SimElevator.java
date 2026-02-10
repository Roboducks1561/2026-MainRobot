package frc.robot.subsystems.defaultSystems.position;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SimElevator extends SubsystemBase implements PositionIO{

    //Set what PID vals to use
    private PIDController pidController;

    private double targetPosition = 0;

    private boolean stopped = false;

    private boolean voltageOut = false;
    private double outputVolts = 0;

    private Thread updateThread;

    //Create elevator with set values
    private final ElevatorSim elevatorSim;
  
    //Main loop of Elevator
    public SimElevator(ElevatorSim elevatorSim, PIDController pidController){
        this.elevatorSim = elevatorSim;
        this.pidController = pidController;
        updateThread = new Thread(()->{
            while(true){
                try {
                    if (stopped){
                        //Stop Elevator
                        elevatorSim.setInputVoltage(0);
                    }else{
                        //Move elevator based on positions
                        if (voltageOut){
                            elevatorSim.setInputVoltage(outputVolts);
                        }else{
                            elevatorSim.setInputVoltage(this.pidController.calculate(getPosition(), targetPosition));
                        }
                    }
                    //Update vals every 20 ms
                    elevatorSim.update(.02);
                    //Sleep every 20 ms
                    Thread.sleep(20);
                } catch (InterruptedException e) {
                    //If exception caught, print StackTrace of exceptions
                    e.printStackTrace();
                }
            }
        });
        updateThread.setDaemon(true);
        updateThread.start();
    }
    @Override
    public void setPosition(double position) {
        targetPosition = position;
        stopped = false;
        voltageOut = false;
    }
    
    @Override
    public void stop() {
        stopped = true;
    }

    @Override
    public double getPosition() {
        return elevatorSim.getPositionMeters();
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

    @Override
    public void setZero() {
        elevatorSim.setState(0,0);
    }
}
