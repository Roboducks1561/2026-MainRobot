package frc.robot.subsystems.intakeMechanism;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constants.IntakeConstants;

import frc.robot.subsystems.defaultSystems.velocity.SimRoller;
import frc.robot.subsystems.defaultSystems.velocity.TalonRoller;
import frc.robot.subsystems.defaultSystems.velocity.VelocityIO;

public class Intake extends SubsystemBase{
    private final VelocityIO intakeIO;

    private final NetworkTable robot = NetworkTableInstance.getDefault().getTable("Robot");
    private final NetworkTable intakeTable = robot.getSubTable("Intake");

    private final DoublePublisher intakeVelocityPublisher = intakeTable
        .getDoubleTopic("IntakeVelocity").publish();
    private final DoublePublisher intakeTargetPublisher = intakeTable
        .getDoubleTopic("IntakeTargetVelocity").publish();

    // private final DigitalInputIO colorSensor;
    // private final DigitalInputIO motorStrain;

    private final double maxError = 1;

    public Intake(){
        if (Robot.isSimulation()){
            intakeIO = new SimRoller(IntakeConstants.intakeSim, new PIDController(20, 0, 0));
            // colorSensor = new DigitalInputSim();
            // motorStrain = new DigitalInputSim();
        }else{
            intakeIO = new TalonRoller(new TalonFX(IntakeConstants.INTAKE_MOTOR_ID), IntakeConstants.talonFXConfiguration, true);
            // colorSensor = new ColorSensor(Port.kOnboard);
            // ((ColorSensor)colorSensor).assignBooleanSupplier(()->{
            //     RawColor col = ((ColorSensor)colorSensor).getColor();
            //     return col.red > 180 || col.blue > 170;
            // });
            // motorStrain = new MotorDI(()->intakeIO.getCurrent(),10);
        }
    }

    public void setVelocity(double rps){
        intakeIO.setVelocity(rps);
    }
    
    //set velocity in rotations
    public Command reachGoal(double rps){
        return this.run(() -> intakeIO.setVelocity(rps));
    }

    //set velocity in rotations
    public Command reachGoal (DoubleSupplier rps){
        return this.run(() -> intakeIO.setVelocity(rps.getAsDouble()));
    }

    public Command setVoltage(DoubleSupplier volts){
        return this.run(()->intakeIO.setVoltage(volts.getAsDouble()));
    }
    
    public Command setVoltage(double volts){
        return this.run(()->intakeIO.setVoltage(volts));
    }

    /**
   * Run control loop to reach and maintain goal.
   *
   * @param goal the position to maintain
   */
    public Command reachGoalOnce(double goal) {
        return this.runOnce(()->intakeIO.setVelocity(goal));
    }

    public Command stop(){
        return this.runOnce(()->intakeIO.stop());
    }

    public double getVelocity(){
        return intakeIO.getVelocity();
    }

    public double getTargetVelocity(){
        return intakeIO.getTarget();
    }

    // public boolean hasPiece(){
    //     return colorSensor.getValue();
    // }

    // public boolean intaking(){
    //     return motorStrain.getValue();
    // }

    public boolean withinBounds(){
        return Math.abs(getTargetVelocity() - getVelocity()) < maxError;
    }

    // public DigitalInputIO getDigitalInputIO(){
    //     return colorSensor;
    // }

    // public DigitalInputIO getMotorStrainIO(){
    //     return motorStrain;
    // }

    @Override
    public void periodic() {
        intakeVelocityPublisher.accept(getVelocity());
        intakeTargetPublisher.accept(getTargetVelocity());
    }
}
