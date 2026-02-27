// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.TurretMechanism;

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
import frc.robot.constants.TurretConstants;
import frc.robot.subsystems.defaultSystems.position.PositionIO;
import frc.robot.subsystems.defaultSystems.position.SimArm;
import frc.robot.subsystems.defaultSystems.position.TalonPosition;


public class Turret extends SubsystemBase {
  
  private final PositionIO armIO;

  //Send Arm data to NetworkTable
  private final NetworkTable robot = NetworkTableInstance.getDefault().getTable("Robot");
  private final NetworkTable armTable = robot.getSubTable("Turret");

  private final StructPublisher<Pose3d> armPublisher = armTable
    .getStructTopic("TurretAngle", Pose3d.struct).publish();

  private final DoublePublisher armRotations = armTable
    .getDoubleTopic("TurretRotations").publish();
  private final DoublePublisher armTarget = armTable
    .getDoubleTopic("TurretTarget").publish();

  public final Transform3d fromSwerveBase = new Transform3d(0,0,.5628, new Rotation3d());

  private final double maxError = .02;

  /** Subsystem constructor. */
  public Turret() {
    if (Robot.isSimulation()){
      armIO = new SimArm(TurretConstants.singleJointedArmSim, new PIDController(100, 0, 1));
    }else{
      armIO = new TalonPosition(
        new TalonFX(TurretConstants.TURRET_MOTOR_ID)
        ,TurretConstants.talonFXConfiguration, true, false
      );//.withEncoder(new CANcoder(TurretConstants.TURRET_ENCODER_ID), TurretConstants.encoderConfiguration);
    }
  }

  public void setPosition(double position){
    armIO.setPosition(MathUtil.clamp(position, Units.radiansToRotations(TurretConstants.MIN_TURRET_ANGLE_RAD), Units.radiansToRotations(TurretConstants.MAX_TURRET_ANGLE_RAD)));
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
    return armIO.getTarget();
  }

  public boolean withinBounds(){
    return Math.abs(getTarget() - getPosition()) < maxError;
  }

  /**
   * 
   * @returns 2 items, first min, second max
   */
  public double[] getMaxPositions(){
    return new double[]{Units.radiansToRotations(TurretConstants.MIN_TURRET_ANGLE_RAD), Units.radiansToRotations(TurretConstants.MAX_TURRET_ANGLE_RAD)};
  }

  public void setZero(){
    armIO.setZero();
  }

  @Override
  public void periodic(){
    armPublisher.accept(new Pose3d(0.12, 0, 0.45,new Rotation3d(0,0,Units.rotationsToRadians(getPosition()))));
    armRotations.accept(getPosition());
    armTarget.accept(getTarget());
  }
}
