// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climbMechanism;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.TurretConstants;
import frc.robot.subsystems.defaultSystems.position.PositionIO;
import frc.robot.subsystems.defaultSystems.position.SimArm;
import frc.robot.subsystems.defaultSystems.position.SimElevator;
import frc.robot.subsystems.defaultSystems.position.TalonPosition;


public class ClimbElevator extends SubsystemBase {
  
  private final PositionIO elevatorIO;

  //Send Arm data to NetworkTable
  private final NetworkTable robot = NetworkTableInstance.getDefault().getTable("Robot");
  private final NetworkTable elevatorTable = robot.getSubTable("Elevator");

  private final StructPublisher<Pose3d> elevatorPublisher = elevatorTable
    .getStructTopic("ElevatorHeight", Pose3d.struct).publish();

  private final DoublePublisher elevatorRotations = elevatorTable
    .getDoubleTopic("ElevatorRotations").publish();
  private final DoublePublisher elevatorTarget = elevatorTable
    .getDoubleTopic("Elevatortarget").publish();

  private final double maxError = .02;

  /** Subsystem constructor. */
  public ClimbElevator() {
    if (Robot.isSimulation()){
      elevatorIO = new SimElevator(ElevatorConstants.elevatorSim, new PIDController(50, 0, 1));
    }else{
      elevatorIO = new TalonPosition(
        new TalonFX(ElevatorConstants.ELEVATOR_LEADER_ID)
        ,ElevatorConstants.talonFXConfiguration, false
      );
    }
  }

  public void setPosition(double position){
    elevatorIO.setPosition(position);
  }

  /**
   * Run control loop to reach and maintain goal.
   *
   * @param goal the position to maintain
   */
  public Command reachGoal(double goal) {
    return this.run(()->setPosition(goal));
  }

  /**
   * Run control loop to reach and maintain changing goal.
   *
   * @param goal the position to maintain
   */
  public Command reachGoal(DoubleSupplier goal) {
    return this.run(()->setPosition(goal.getAsDouble()));
  }

  /**
   * Run control loop to reach and maintain goal.
   *
   * @param goal the position to maintain
   */
  public Command reachGoalOnce(double goal) {
    return this.runOnce(()->setPosition(goal));
  }

  public Command setVoltage(DoubleSupplier volts){
    return this.run(()->elevatorIO.setVoltage(volts.getAsDouble()));
  }

  public Command setVoltage(double volts){
    return this.run(()->elevatorIO.setVoltage(volts));
  }

  public Command stop(){
    return this.runOnce(()->elevatorIO.stop());
  }

  //Get position of Arm
  public double getPosition(){
    return elevatorIO.getPosition();
  }

  //Get target
  public double getTarget(){
    return elevatorIO.getTarget();
  }

  public boolean withinBounds(){
    return Math.abs(getTarget() - getPosition()) < maxError;
  }

  public void setZero(){
    elevatorIO.setZero();
  }
  
  @Override
  public void periodic(){
    elevatorPublisher.accept(new Pose3d(0.12, 0, 0.45,new Rotation3d(0,0,Units.rotationsToRadians(getPosition()))));
    elevatorRotations.accept(getPosition());
    elevatorTarget.accept(getTarget());
  }
}
