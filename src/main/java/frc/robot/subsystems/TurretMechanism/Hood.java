// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.TurretMechanism;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.networktables.StructSubscriber;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constants.HoodConstants;
import frc.robot.subsystems.defaultSystems.position.PositionIO;
import frc.robot.subsystems.defaultSystems.position.SimArm;
import frc.robot.subsystems.defaultSystems.position.TalonPosition;


public class Hood extends SubsystemBase {
  
  private final PositionIO armIO;

  //Send Arm data to NetworkTable
  private final NetworkTable robot = NetworkTableInstance.getDefault().getTable("Robot");
  private final NetworkTable armTable = robot.getSubTable("Hood");

  private final StructPublisher<Pose3d> armPublisher = armTable
    .getStructTopic("HoodAngle", Pose3d.struct).publish();

  private final DoublePublisher armRotations = armTable
    .getDoubleTopic("HoodRotations").publish();
  private final DoublePublisher armTarget = armTable
    .getDoubleTopic("HoodTarget").publish();

  private final double maxError = .005;

  /** Subsystem constructor. */
  public Hood() {
    if (Robot.isSimulation()){
      armIO = new SimArm(HoodConstants.singleJointedArmSim, new PIDController(110, 0, 7));
    }else{
      armIO = new TalonPosition(
        new TalonFX(HoodConstants.HOOD_MOTOR_ID,"Canivore")
        ,HoodConstants.talonFXConfiguration, true
      );
    }
  }

  public void setPosition(double position){
    armIO.setPosition(MathUtil.clamp(position, Units.radiansToRotations(HoodConstants.MIN_HOOD_ANGLE_RAD), Units.radiansToRotations(HoodConstants.MAX_HOOD_ANGLE_RAD)));
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
    return this.run(()->armIO.setVoltage(volts.getAsDouble()));
  }

  public Command setVoltage(double volts){
    return this.run(()->armIO.setVoltage(volts));
  }

  public Command stop(){
    return this.runOnce(()->armIO.stop());
  }

  //Get position of Arm
  public double getPosition(){
    return armIO.getPosition();
  }

  //Get target
  public double getTarget(){
    //TODO Wrong
    return armIO.getTarget();
  }

  public boolean withinBounds(){
    return Math.abs(getTarget() - getPosition()) < maxError;
  }

  public void setZero(){
    armIO.setZero();
  }

  @Override
  public void periodic(){
    double minimumHoodRotation = .02;
    // armPublisher.accept(new Pose3d());
    armPublisher.accept(new Pose3d(-.16, 0, .398,new Rotation3d(0,Units.rotationsToRadians(getPosition()+minimumHoodRotation),0)));
    armRotations.accept(getPosition());
    armTarget.accept(getTarget());
  }
}