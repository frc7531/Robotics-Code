/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.io.Console;

import com.ctre.phoenix.*;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxRelativeEncoder.Type;

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

  CANSparkMax fieldMotor1 = new CANSparkMax(7, MotorType.kBrushless);
  CANSparkMax fieldMotor2 = new CANSparkMax(8, MotorType.kBrushless);

  RelativeEncoder e1 = rightMotor1.getEncoder(Type.kHallSensor, 42);
  RelativeEncoder e2 = leftMotor1.getEncoder(Type.kHallSensor, 42);

  Joystick leftStick = new Joystick(0);
  Joystick rightStick = new Joystick(1);
  Joystick driver2 = new Joystick(2);

  AddressableLED leds = new AddressableLED(0);
  AddressableLEDBuffer buffer = new AddressableLEDBuffer(158);

  Compressor c = new Compressor(10, PneumaticsModuleType.CTREPCM);
  DoubleSolenoid rightClaw = new DoubleSolenoid(10, PneumaticsModuleType.CTREPCM, 0, 1);
  DoubleSolenoid leftClaw = new DoubleSolenoid(10, PneumaticsModuleType.CTREPCM, 2, 3);
  DoubleSolenoid armMain = new DoubleSolenoid(10, PneumaticsModuleType.CTREPCM, 4, 5);
  DoubleSolenoid armWrist = new DoubleSolenoid(10, PneumaticsModuleType.CTREPCM, 6, 7);

  Servo s1 = new Servo(1);

  float m_rainbowFirstPixelHue = 0;

  private boolean idleMode = false; // false = coast, true = brake

  int color = 0;

  @Override
  public void autonomousInit() {
    leftMotor1.setIdleMode(IdleMode.kBrake);
    leftMotor2.setIdleMode(IdleMode.kBrake);
    leftMotor3.setIdleMode(IdleMode.kBrake);
    rightMotor1.setIdleMode(IdleMode.kBrake);
    rightMotor2.setIdleMode(IdleMode.kBrake);
    rightMotor3.setIdleMode(IdleMode.kBrake);
    System.out.println("Robot took " + moveStraight(10, 0.1, 0.01) + " seconds to move");
  }

  public void autonomousPeriodic() {
  }

  @Override
  public void robotInit() {
    leds.setLength(buffer.getLength());
    leds.setData(buffer);
    leds.start();

    rightMotor1.setInverted(true);
    rightMotor2.setInverted(true);
    rightMotor3.setInverted(true);

    fieldMotor1.setIdleMode(IdleMode.kBrake);
    fieldMotor2.setIdleMode(IdleMode.kBrake);

    //armMain.set(Value.kReverse);
    //armWrist.set(Value.kForward);
  }

  @Override
  public void teleopInit() {
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
    fieldMotor1.set(driver2.getRawAxis(1) / 2);
    fieldMotor2.set(driver2.getRawAxis(1) / 2);
    drive(leftStick.getY(), rightStick.getY(), true, 1000);
    if(driver2.getRawButton(6)) rightClaw.set(Value.kReverse);
    if(driver2.getRawButton(5)) leftClaw.set(Value.kForward);
    if(driver2.getRawAxis(3) > 0.75) rightClaw.set(Value.kForward);
    if(driver2.getRawAxis(2) > 0.75) leftClaw.set(Value.kReverse);
    if(driver2.getPOV() == 0) {
      armWrist.set(Value.kForward);
      Timer.delay(1);
      armMain.set(Value.kReverse);
      Timer.delay(1);
      armWrist.set(Value.kReverse);
    }
    if(driver2.getPOV() == 180) {
      armWrist.set(Value.kForward);
      armMain.set(Value.kForward);
      Timer.delay(1);
      armWrist.set(Value.kReverse);
    }
  }

  @Override
  public void robotPeriodic() {

    if(leftStick.getRawButton(3)) {
      color = 1;
    }
    if(leftStick.getRawButton(4)) {
      color = 0;
    }
    if(leftStick.getRawButton(5)) {
      color = 2;
    }
    
    switch(color) {
      case 1:
      rainbow();
      break;
      case 0:
      for(int i = 0; i < buffer.getLength(); i++) {
        buffer.setRGB(i, 255, 0, 0);
      }
      break;
      case 2:
      for(int i = 0; i < buffer.getLength(); i++) {
        buffer.setRGB(i, 0, 0, 255);
      }
      break;
    }
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
      rightSpeed = -rightSquare;
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

  private double moveStraight(double distance, double speed, double precision) {
    Timer timer = new Timer();
    timer.reset();
    timer.start();
    speed = Math.abs(speed);
    precision = Math.abs(precision);
    boolean state1 = true;
    boolean state2 = true;
    e1.setPosition(0);
    e2.setPosition(0);
    while(state1 || state2) {
      if(e1.getPosition() < distance - precision) {
        state1 = true;
        rightMotor1.set(speed);
        rightMotor2.set(speed);
        rightMotor3.set(speed);
      } else if(e1.getPosition() > distance + precision) {
        state1 = true;
        rightMotor1.set(-speed);
        rightMotor2.set(-speed);
        rightMotor3.set(-speed);
      } else {
        rightMotor1.set(0);
        rightMotor2.set(0);
        rightMotor3.set(0);
        state1 = false;
      }
      if(e2.getPosition() < distance - precision) {
        state2 = true;
        leftMotor1.set(speed);
        leftMotor2.set(speed);
        leftMotor3.set(speed);
      } else if(e1.getPosition() > distance + precision) {
        state2 = true;
        leftMotor1.set(-speed);
        leftMotor2.set(-speed);
        leftMotor3.set(-speed);
      } else {
        leftMotor1.set(0);
        leftMotor2.set(0);
        leftMotor3.set(0);
        state2 = false;
      }
    }
    timer.stop();
    return timer.get();
  }

  private void rainbow() {
    // For every pixel
    for (var i = 0; i < buffer.getLength(); i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      final var hue = (m_rainbowFirstPixelHue + (i * 720 / buffer.getLength())) % 180;
      // Set the value
      buffer.setHSV(i, (int)hue, 255, 128);
    }
    // Increase by to make the rainbow "move"
    m_rainbowFirstPixelHue += 1;
    // Check bounds
    m_rainbowFirstPixelHue %= 180;
  }
}
