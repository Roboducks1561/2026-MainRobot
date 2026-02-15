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

}
