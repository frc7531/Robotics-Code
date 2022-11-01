// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.MotorSafety;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.*;

import java.util.ArrayList;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Robot extends TimedRobot {
  private CANSparkMax[] motors;
  private Translation2d[] translations;
  private DutyCycleEncoder[] encoders;
  private MotorGroup[] motorGroups;
  private SwerveDriveKinematics sdk;
  private SwerveDrive swerve;

  private ADXRS450_Gyro gyro;

  private Joystick leftJoystick;
  private Joystick rightJoystick;
  private Joystick classicController;
  private Joystick wiimote;
  private Joystick nunchuk;

  private ArrayList<Double> joystickAverage0;
  private ArrayList<Double> joystickAverage1;
  private ArrayList<Double> joystickAverage2;

  private IdleMode mode = IdleMode.kCoast;

  @Override public void robotInit() {
    // CameraServer.startAutomaticCapture();
    translations = new Translation2d[] {
      new Translation2d(0.5, 0.5),
      new Translation2d(0.5, -0.5),
      new Translation2d(-0.5, -0.5),
      new Translation2d(-0.5, 0.5),
    };
    motors = new CANSparkMax[] {
      new CANSparkMax(1, MotorType.kBrushless),
      new CANSparkMax(2, MotorType.kBrushless),
      new CANSparkMax(3, MotorType.kBrushless),
      new CANSparkMax(4, MotorType.kBrushless),
      new CANSparkMax(5, MotorType.kBrushless),
      new CANSparkMax(6, MotorType.kBrushless),
      new CANSparkMax(7, MotorType.kBrushless),
      new CANSparkMax(8, MotorType.kBrushless),
    };
    encoders = new DutyCycleEncoder[] {
      new DutyCycleEncoder(0),
      new DutyCycleEncoder(1),
      new DutyCycleEncoder(2),
      new DutyCycleEncoder(3),
    };
    motorGroups = new MotorGroup[4];
    for(int i = 0; i < motorGroups.length; i++) {
      motorGroups[i] = new MotorGroup(motors[2*i], motors[2*i + 1], encoders[i]);
    }
    for(int i = 0; i < 8; i++) {
      motors[i].setIdleMode(mode);
      motors[i].getEncoder().setVelocityConversionFactor(2 / (double)15);
    }
    sdk = new SwerveDriveKinematics(translations);
    
    gyro = new ADXRS450_Gyro();
    swerve = new SwerveDrive(motorGroups, sdk, gyro);

    leftJoystick = new Joystick(0);
    rightJoystick = new Joystick(1);
    wiimote = new Joystick(3);
    classicController = new Joystick(4);
    nunchuk = new Joystick(5);

    joystickAverage0 = new ArrayList<Double>();
    joystickAverage1 = new ArrayList<Double>();
    joystickAverage2 = new ArrayList<Double>();
  }
  @Override public void robotPeriodic() {
    if(leftJoystick.getRawButtonPressed(6)) {
      for(int i = 0; i < 8; i++) {
        motors[i].set(0);
      }
      try {
        Thread.sleep(100);
      } catch(InterruptedException e) {
        e.printStackTrace();
      }
      gyro.calibrate();
    }
  }

  @Override public void autonomousInit() {
    public SwerveControllerCommand
  }

  @Override public void autonomousPeriodic() {}
  @Override public void testInit() {}

  @Override public void testPeriodic() {
    if(classicController.getRawButton(1)) {
      swerve.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
      -0,
      -0.3, 
      0, 
      Rotation2d.fromDegrees(0)), 
      1.0, 
      0.00);
    } else {
      for (CANSparkMax motor : motors) {
        motor.set(0);
      }
    }
  }

  @Override public void teleopInit() {}

  @Override public void teleopPeriodic() {

     swerve.drive(/*ChassisSpeeds.fromFieldRelativeSpeeds(
      -leftJoystick.getX() / 3, 
      -leftJoystick.getY() / 3, 
      rightJoystick.getX() / 3, 
      Rotation2d.fromDegrees(gyro.getAngle()))*/
      new ChassisSpeeds(-leftJoystick.getX() / 3, -leftJoystick.getY() / 3, rightJoystick.getX() / 3), 
      1.0, 
      0.00);
  }

  @Override public void disabledInit() {}
  @Override public void disabledPeriodic() {}

  private double smoothInput(int id, double input) {
    ArrayList<Double> temparr = null;
    switch(id) {
      case 0:
        temparr = joystickAverage0;
        break;
      case 1:
        temparr = joystickAverage1;
        break;
      case 2:
        temparr = joystickAverage2;
        break;
      default:
        return -1;
    }
    temparr.add(0, input);
    if(temparr.size() >= 10) {
      temparr.remove(temparr.size() - 1);
    }
    double averageTotal = 0;
    for(int i = 0; i < temparr.size(); i++){
      averageTotal += temparr.get(i);
    }
    double average = averageTotal / temparr.size();
    return average;
  }
}
