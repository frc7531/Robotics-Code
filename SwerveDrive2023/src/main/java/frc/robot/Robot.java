// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.*;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Robot extends TimedRobot {
  CANSparkMax[] motors;
  Translation2d[] translations;
  DutyCycleEncoder[] encoders;
  MotorGroup[] motorGroups;
  SwerveDriveKinematics sdk;
  SwerveDrive swerve;

  Joystick controller;

  @Override
  public void robotInit() {
    translations = new Translation2d[] {
      new Translation2d(-0.5, 0.5),
      new Translation2d(0.5, 0.5),
      new Translation2d(0.5, -0.5),
      new Translation2d(-0.5, -0.5)
    };
    motors = new CANSparkMax[] {
      new CANSparkMax(0, MotorType.kBrushless),
      new CANSparkMax(1, MotorType.kBrushless),
      new CANSparkMax(2, MotorType.kBrushless),
      new CANSparkMax(3, MotorType.kBrushless),
      new CANSparkMax(4, MotorType.kBrushless),
      new CANSparkMax(5, MotorType.kBrushless),
      new CANSparkMax(6, MotorType.kBrushless),
      new CANSparkMax(7, MotorType.kBrushless),
      new CANSparkMax(8, MotorType.kBrushless)
    };
    encoders = new DutyCycleEncoder[] {
      new DutyCycleEncoder(0),
      new DutyCycleEncoder(1),
      new DutyCycleEncoder(2),
      new DutyCycleEncoder(3)
    };
    motorGroups = new MotorGroup[4];
    for(int i = 0; i < 4; i++) {
      motorGroups[i] = new MotorGroup(motors[i], motors[i + 1], encoders[i]);
    }
    sdk = new SwerveDriveKinematics(translations);
    swerve = new SwerveDrive(motorGroups, sdk);

    controller = new Joystick(0);
  }
  @Override
  public void robotPeriodic() {

  }
  @Override
  public void autonomousInit() {

  }
  @Override
  public void autonomousPeriodic() {

  }
  @Override
  public void teleopInit() {

  }
  @Override
  public void teleopPeriodic() {
    swerve.drive(new ChassisSpeeds(controller.getRawAxis(0), controller.getRawAxis(1), controller.getRawAxis(3)), 1, 5);
  }
  @Override
  public void disabledInit() {

  }
  @Override
  public void disabledPeriodic() {

  }
}