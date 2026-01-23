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
import frc.robot.constants.SpindexerConstants;
import frc.robot.subsystems.defaultSystems.digitalInputs.CANRange;
import frc.robot.subsystems.defaultSystems.digitalInputs.DigitalInputIO;
import frc.robot.subsystems.defaultSystems.digitalInputs.DigitalInputSim;
import frc.robot.subsystems.defaultSystems.velocity.SimRoller;
import frc.robot.subsystems.defaultSystems.velocity.TalonRoller;
import frc.robot.subsystems.defaultSystems.velocity.VelocityIO;

public class Spindexer extends SubsystemBase{
    private final VelocityIO rollerIO;

    private final NetworkTable robot = NetworkTableInstance.getDefault().getTable("Robot");
    private final NetworkTable IndexerTable = robot.getSubTable("Spindexer");

    private final DoublePublisher rollerVelocityPublisher = IndexerTable
        .getDoubleTopic("SpindexerVelocity").publish();
    private final DoublePublisher rollerTargetPublisher = IndexerTable
        .getDoubleTopic("SpindexerTargetVelocity").publish();

    private final DigitalInputIO canRange;

    private final double maxError = 1;

    public Spindexer(){
        if (Robot.isSimulation()){
            rollerIO = new SimRoller(SpindexerConstants.RollerSim, new PIDController(20, 0, 0));
            canRange = new DigitalInputSim();
        }else{
            rollerIO = new TalonRoller(new TalonFX(SpindexerConstants.INDEXER_MOTOR_ID), SpindexerConstants.talonFXConfiguration, false);
            canRange = new CANRange(SpindexerConstants.INDEXER_CAN_RANGE_ID, .1, "");
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

    public boolean hasPiece(){
        return canRange.getValue();
    }

    public boolean withinBounds(){
        return Math.abs(getTargetVelocity() - getVelocity()) < maxError;
    }

    public DigitalInputIO getDigitalInputIO(){
        return canRange;
    }

    @Override
    public void periodic() {
        rollerVelocityPublisher.accept(getVelocity());
        rollerTargetPublisher.accept(getTargetVelocity());
    }
}
