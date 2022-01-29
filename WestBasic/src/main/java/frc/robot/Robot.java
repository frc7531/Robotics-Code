/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
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

  AddressableLED leds = new AddressableLED(0);
  AddressableLEDBuffer buffer = new AddressableLEDBuffer(158);

  int m_rainbowFirstPixelHue = 0;

  private boolean idleMode = false; // false = coast, true = brake

  @Override
  public void robotInit() {

    leds.setLength(buffer.getLength());
    leds.setData(buffer);
    leds.start();

    if(idleMode) {
      //set drive motors to brake when idle
      leftMotor1.setIdleMode(IdleMode.kBrake);
      leftMotor2.setIdleMode(IdleMode.kBrake);
      leftMotor3.setIdleMode(IdleMode.kBrake);
      rightMotor1.setIdleMode(IdleMode.kBrake);
      rightMotor2.setIdleMode(IdleMode.kBrake);
      rightMotor3.setIdleMode(IdleMode.kBrake);
    } else {
      //set drive motors to coast when idle
      leftMotor1.setIdleMode(IdleMode.kCoast);
      leftMotor2.setIdleMode(IdleMode.kCoast);
      leftMotor3.setIdleMode(IdleMode.kCoast);
      rightMotor1.setIdleMode(IdleMode.kCoast);
      rightMotor2.setIdleMode(IdleMode.kCoast);
      rightMotor3.setIdleMode(IdleMode.kCoast);
    }
  }

  @Override
  public void teleopPeriodic() {
    drive(leftStick.getY(), rightStick.getY(), true, 1000);
    for(int i = 0; i < buffer.getLength(); i++) {
      buffer.setHSV(i, i, 50, 50);
    }
  }

  @Override
  public void robotPeriodic() {
    rainbow();
    leds.setData(buffer);
  }

  /**Custom method to drive the robot in tank drive mode, with square input mode
   * @param left_speed The speed of the left motors
   * @param right_speed The speed of the right motors
   * @param squareInputs Whether or not to square the inputs before driving(makes it easier to control at lower speeds)
   * @param deadZone how big of a dead zone to have on the joysticks, bigger number means smaller, recommended 20
  */
  private void drive(double left_speed, double right_speed, boolean squareInputs, float deadZone) {
    double leftSquare = Math.copySign(Math.pow(left_speed, 2), left_speed);
    double rightSquare = Math.copySign(Math.pow(right_speed, 2), right_speed);

    double leftSpeed;
    double rightSpeed;

    if(squareInputs) {
      leftSpeed = -leftSquare;
      rightSpeed = rightSquare;
    } else {
      leftSpeed = left_speed;
      rightSpeed = right_speed;
    }

    if((int)(leftSpeed * deadZone) != 0) {
      leftMotor1.set(leftSpeed);
      leftMotor2.set(leftSpeed);
      leftMotor3.set(leftSpeed);
    } else { leftMotor1.set(0);  leftMotor2.set(0);  leftMotor3.set(0); }

    if((int)(rightSpeed * deadZone) != 0) {
      rightMotor1.set(rightSpeed);
      rightMotor2.set(rightSpeed);
      rightMotor3.set(rightSpeed);
    } else { rightMotor1.set(0);  rightMotor2.set(0);  rightMotor3.set(0); }
  }

  private void rainbow() {
    // For every pixel
    for (var i = 0; i < buffer.getLength(); i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      final var hue = (m_rainbowFirstPixelHue + (i * 180 / buffer.getLength())) % 180;
      // Set the value
      buffer.setHSV(i, hue, 255, 128);
    }
    // Increase by to make the rainbow "move"
    m_rainbowFirstPixelHue += 1;
    // Check bounds
    m_rainbowFirstPixelHue %= 180;
  }
}
