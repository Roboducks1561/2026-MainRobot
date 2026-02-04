// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intakeMechanism;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constants.ArmConstants;
import frc.robot.subsystems.defaultSystems.position.PositionIO;
import frc.robot.subsystems.defaultSystems.position.SimArm;
import frc.robot.subsystems.defaultSystems.position.TalonPosition;


public class Arm extends SubsystemBase {
  
  private final PositionIO armIO1;
  private final PositionIO armIO2;

  //Send Arm data to NetworkTable
  private final NetworkTable robot = NetworkTableInstance.getDefault().getTable("Robot");
  private final NetworkTable armTable = robot.getSubTable("Arm");

  private final StructPublisher<Pose3d> armPublisher = armTable
    .getStructTopic("ArmAngle", Pose3d.struct).publish();
  private final StructPublisher<Pose3d> hopperPublisher = armTable
    .getStructTopic("HopperPosition", Pose3d.struct).publish();

  private final DoublePublisher armRotations = armTable
    .getDoubleTopic("ArmRotations").publish();
  private final DoublePublisher armTarget = armTable
    .getDoubleTopic("ArmTarget").publish();

  private final double maxError = .02;

  /** Subsystem constructor. */
  public Arm() {
    if (Robot.isSimulation()){
      armIO1 = new SimArm(ArmConstants.singleJointedArmSim, new PIDController(50, 0, 3));
      armIO2 = new SimArm(ArmConstants.singleJointedArmSim, new PIDController(50, 0, 3));
    }else{
      armIO1 = new TalonPosition(
        new TalonFX(ArmConstants.ARM_MOTOR_LEFT_ID)
        ,ArmConstants.talonFXConfiguration, false
      )/*.withFollower(new TalonFX(ArmConstants.ARM_MOTOR_RIGHT_ID), false)*/;
      armIO2 = new TalonPosition(
        new TalonFX(ArmConstants.ARM_MOTOR_RIGHT_ID)
        ,ArmConstants.talonFXConfiguration, false
      );
    }
  }

  public void setPosition(double position){
    armIO1.setPosition(position);
    armIO2.setPosition(position);
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
    return this.run(()->setVoltage(volts.getAsDouble()));
  }

  public Command setVoltage(double volts){
    return this.run(()->setVoltage(volts));
  }

  public Command stop(){
    return this.runOnce(()->{
      armIO1.stop();
      armIO2.stop();
    });
  }

  //Get position of Arm
  public double getPosition(){
    return armIO1.getPosition();
  }

  //Get target
  public double getTarget(){
    return armIO1.getTarget();
  }

  public boolean withinBounds(){
    return Math.abs(getTarget() - getPosition()) < maxError;
  }

  @Override
  public void periodic(){
    // armPublisher.accept(new Pose3d());
    armPublisher.accept(new Pose3d(0.2,0, 0.15,new Rotation3d(0,Units.rotationsToRadians(getPosition()),0)));
    double meterRadius = .172;
    hopperPublisher.accept(new Pose3d(Math.sin(Units.rotationsToRadians(getPosition()))*meterRadius,0, 0,new Rotation3d(0,0,0)));
    armRotations.accept(getPosition());
    armTarget.accept(getTarget());
  }
}
