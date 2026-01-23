package frc.robot.subsystems.TurretMechanism;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.defaultSystems.velocity.SimRoller;
import frc.robot.subsystems.defaultSystems.velocity.TalonRoller;
import frc.robot.subsystems.defaultSystems.velocity.VelocityIO;

public class Shooter extends SubsystemBase{
    private final VelocityIO rollerIO;

    private final NetworkTable robot = NetworkTableInstance.getDefault().getTable("Robot");
    private final NetworkTable rollerTable = robot.getSubTable("Shooter");

    private final DoublePublisher rollerVelocityPublisher = rollerTable
        .getDoubleTopic("ShooterVelocity").publish();
    private final DoublePublisher rollerTargetPublisher = rollerTable
        .getDoubleTopic("ShooterTargetVelocity").publish();

    private final double maxError = 1;

    public Shooter(){
        if (Robot.isSimulation()){
            rollerIO = new SimRoller(ShooterConstants.shooterSim, new PIDController(20, 0, 0));
        }else{
            rollerIO = new TalonRoller(new TalonFX(ShooterConstants.SHOOTER_MOTOR_ID, "Canivore"), ShooterConstants.talonFXConfiguration, false);
        }
    }

    public void setVelocity(double rps){
        rollerIO.setVelocity(rps);
    }
    
    //set velocity in rotations
    public Command reachGoal(double rps){
        return this.run(() -> rollerIO.setVelocity(rps));
    }

    //set velocity in rotations
    public Command reachGoal (DoubleSupplier rps){
        return this.run(() -> rollerIO.setVelocity(rps.getAsDouble()));
    }

    public Command setVoltage(DoubleSupplier volts){
        return this.run(()->rollerIO.setVoltage(volts.getAsDouble()));
    }
    
    public Command setVoltage(double volts){
        return this.run(()->rollerIO.setVoltage(volts));
    }

    /**
   * Run control loop to reach and maintain goal.
   *
   * @param goal the position to maintain
   */
    public Command reachGoalOnce(double goal) {
        return this.runOnce(()->rollerIO.setVelocity(goal));
    }

    public Command stop(){
        return this.runOnce(()->rollerIO.stop());
    }

    public double getVelocity(){
        return rollerIO.getVelocity();
    }

    public double getTargetVelocity(){
        return rollerIO.getTarget();
    }

    public boolean withinBounds(){
        return Math.abs(getTargetVelocity() - getVelocity()) < maxError;
    }

    @Override
    public void periodic() {
        rollerVelocityPublisher.accept(getVelocity());
        rollerTargetPublisher.accept(getTargetVelocity());
    }
}
