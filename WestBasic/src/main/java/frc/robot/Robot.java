/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.REVLibError;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.cameraserver.CameraServer;

/**
 * This is a demo program showing the use of the CANSparkMax class, specifically
 * it contains the code necessary to operate a robot with tank drive.
 */
public class Robot extends TimedRobot {

  CANSparkMax rightMotor1 = new CANSparkMax(1, MotorType.kBrushless);
  CANSparkMax rightMotor2 = new CANSparkMax(2, MotorType.kBrushless);
  CANSparkMax rightMotor3 = new CANSparkMax(3, MotorType.kBrushless);
  
  CANSparkMax leftMotor1 = new CANSparkMax(4, MotorType.kBrushless);
  CANSparkMax leftMotor2 = new CANSparkMax(5, MotorType.kBrushless);
  CANSparkMax leftMotor3 = new CANSparkMax(6, MotorType.kBrushless);

  Joystick leftStick = new Joystick(0);
  Joystick rightStick = new Joystick(1);

  @Override
  public void robotInit() {
    CameraServer.startAutomaticCapture();
  }

  @Override
  public void teleopPeriodic() {
    drive(leftStick.getY(), rightStick.getY());
  }

  public void drive(double left_speed, double right_speed) {
    if((int)(left_speed * 20) != 0) {
      leftMotor1.set(-Math.copySign(left_speed * left_speed, left_speed));
      leftMotor2.set(-Math.copySign(left_speed * left_speed, left_speed));
      leftMotor3.set(-Math.copySign(left_speed * left_speed, left_speed));
    } else { leftMotor1.set(0);  leftMotor2.set(0);  leftMotor3.set(0); }
    if((int)(right_speed * 20) != 0) {
      rightMotor1.set(Math.copySign(right_speed * right_speed, right_speed));
      rightMotor2.set(Math.copySign(right_speed * right_speed, right_speed));
      rightMotor3.set(Math.copySign(right_speed * right_speed, right_speed));
    } else { rightMotor1.set(0);  rightMotor2.set(0);  rightMotor3.set(0); }
  }
}
