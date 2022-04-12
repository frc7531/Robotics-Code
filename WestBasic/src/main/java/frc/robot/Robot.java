/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

//Robot package
package frc.robot;
//WPILIB imports
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
//REV imports
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxRelativeEncoder.Type;
//all the code really goes in here
public class Robot extends TimedRobot {
  //Create objects for each of the motors, all SparkMAX's
  private CANSparkMax rightMotor1 = new CANSparkMax(1, MotorType.kBrushless);
  private CANSparkMax rightMotor2 = new CANSparkMax(2, MotorType.kBrushless);
  private CANSparkMax rightMotor3 = new CANSparkMax(3, MotorType.kBrushless);
  private CANSparkMax leftMotor1 = new CANSparkMax(4, MotorType.kBrushless);
  private CANSparkMax leftMotor2 = new CANSparkMax(5, MotorType.kBrushless);
  private CANSparkMax leftMotor3 = new CANSparkMax(6, MotorType.kBrushless);
  private CANSparkMax fieldMotor1 = new CANSparkMax(7, MotorType.kBrushless);
  private CANSparkMax fieldMotor2 = new CANSparkMax(8, MotorType.kBrushless);
  //Create objects for the encoders on two of the motors -
  //We only need one encoder from each side of the robot,
  //because all the motors on one side are chained together
  private RelativeEncoder e1 = rightMotor1.getEncoder(Type.kHallSensor, 42);
  private RelativeEncoder e2 = leftMotor1.getEncoder(Type.kHallSensor, 42);
  //Create Joystick objects for drivers 1 and 2
  private Joystick leftStick = new Joystick(0);
  private Joystick rightStick = new Joystick(1);
  private Joystick driver2 = new Joystick(2);
  //Create LED objects for the LED strip on the bottom of the robot
  private AddressableLED leds = new AddressableLED(0);
  private AddressableLEDBuffer buffer = new AddressableLEDBuffer(164);
  //Create pneumatics objects
  protected Compressor c = new Compressor(10, PneumaticsModuleType.CTREPCM);
  private DoubleSolenoid rightClaw = new DoubleSolenoid(10, PneumaticsModuleType.CTREPCM, 0, 1);
  private DoubleSolenoid leftClaw = new DoubleSolenoid(10, PneumaticsModuleType.CTREPCM, 2, 3);
  private DoubleSolenoid armMain = new DoubleSolenoid(10, PneumaticsModuleType.CTREPCM, 4, 5);
  private DoubleSolenoid armWrist = new DoubleSolenoid(10, PneumaticsModuleType.CTREPCM, 6, 7);
  private DoubleSolenoid flag = new DoubleSolenoid(11, PneumaticsModuleType.CTREPCM, 0, 1);

  //start hue color of rainbow LEDs
  private float m_rainbowFirstPixelHue = 0;
  //The mode when the motors are idle, either coast or brake
  private boolean idleMode = false; // false = coast, true = brake
  
  //DURING AUTONOMOUS: true is to release ball during autonomous, false is to not release the ball
  private boolean dropBall = false;

  private double speedModifier;

  Timer auto = new Timer();
  //The current color mode of the LED strips
  private enum Color {
    red,
    blue,
    rainbow,
    off
  }

  Color color = Color.off;

  /**This code runs once when the robot initializes */
  @Override
  public void robotInit() {
    //sets up things to do with the LEDs
    leds.setLength(buffer.getLength());
    leds.setData(buffer);
    leds.start();

    e1.setPositionConversionFactor(3.35);
    e2.setPositionConversionFactor(3.35);

    //invert the right motors because they are facing the other direction
    rightMotor1.setInverted(true);
    rightMotor2.setInverted(true);
    rightMotor3.setInverted(true);

    //set the idle mode of the winch motors to brake, keeps it more steady
    //could add an automatic steady system with the encoders to make it stay in exactly the same place
    //actually wouldn't be that hard, requires further testing
    fieldMotor1.setIdleMode(IdleMode.kBrake);
    fieldMotor2.setIdleMode(IdleMode.kBrake);

    //armMain.set(Value.kReverse);
    //armWrist.set(Value.kForward);
  }

  /**Code that runs repeatedly whenever the robot is on and connected
   * All this code does is to update the LED strip while the robot is disabled
   */
  @Override
  public void robotPeriodic() {
    switch(color) {
      case rainbow:
      rainbow();
      break;
      case red:
      for(int i = 0; i < buffer.getLength(); i++)
        buffer.setRGB(i, 255, 0, 0);
      break;
      case blue:
      for(int i = 0; i < buffer.getLength(); i++)
        buffer.setRGB(i, 0, 0, 255);
      break;
      case off:
      for(int i = 0; i < buffer.getLength(); i++)
        buffer.setRGB(i, 0, 0, 0);
      break;
    }
    leds.setData(buffer);
  }

  /**This code runs once when the autonomous period starts */
  @Override
  public void autonomousInit() {
    auto.start();

    color = Color.rainbow;
    //motors braking in autonomous is much more accurate
    leftMotor1.setIdleMode(IdleMode.kBrake);
    leftMotor2.setIdleMode(IdleMode.kBrake);
    leftMotor3.setIdleMode(IdleMode.kBrake);
    rightMotor1.setIdleMode(IdleMode.kBrake);
    rightMotor2.setIdleMode(IdleMode.kBrake);
    rightMotor3.setIdleMode(IdleMode.kBrake);
    
    if(dropBall) {
      flag.set(Value.kForward);
      armWrist.set(Value.kReverse);
      Timer.delay(0.125);
      leftClaw.set(Value.kForward);
      rightClaw.set(Value.kReverse);
      Timer.delay(1);
    }

    moveStraight(-108, 0.1, 0.1);
    e1.setPosition(0);
    e2.setPosition(0);
    armWrist.set(Value.kReverse);
    //flag.set(Value.kReverse);
  }

  /**This code runs repeatedly when the autonomous period is active */
  @Override
  public void autonomousPeriodic() {
    //nothing at the moment
  }

  /**Code that runs once when the teleop period starts */
  @Override
  public void teleopInit() {
    //sets the drive motors to either brake or coast when idle
    flag.set(Value.kForward);
    if(idleMode) {
      leftMotor1.setIdleMode(IdleMode.kBrake);
      leftMotor2.setIdleMode(IdleMode.kBrake);
      leftMotor3.setIdleMode(IdleMode.kBrake);
      rightMotor1.setIdleMode(IdleMode.kBrake);
      rightMotor2.setIdleMode(IdleMode.kBrake);
      rightMotor3.setIdleMode(IdleMode.kBrake);
    } else {
      leftMotor1.setIdleMode(IdleMode.kCoast);
      leftMotor2.setIdleMode(IdleMode.kCoast);
      leftMotor3.setIdleMode(IdleMode.kCoast);
      rightMotor1.setIdleMode(IdleMode.kCoast);
      rightMotor2.setIdleMode(IdleMode.kCoast);
      rightMotor3.setIdleMode(IdleMode.kCoast);
    }
    color = Color.red;
    
    //armMain.set(Value.kForward);
  }

  /**Code that runs repeatedly when the teleop period is active */
  @Override
  public void teleopPeriodic() {
    //sets winch motors to the Y axis of Driver 2's left thumb stick
    fieldMotor1.set(Math.copySign(Math.pow(driver2.getRawAxis(1)/* * 3 / 4*/, 2), -driver2.getRawAxis(1)));
    fieldMotor2.set(Math.copySign(Math.pow(driver2.getRawAxis(1)/* * 3 / 4*/, 2), -driver2.getRawAxis(1)));
    //controls the drive motors and optionally squares the inputs
    drive(leftStick.getY() * speedModifier, rightStick.getY() * speedModifier, true, 1000);

    if(rightStick.getRawButton(2)) speedModifier = 0.5;
    else if(leftStick.getRawButton(2)) speedModifier = 1.0;
    else speedModifier = 2.0/3.0;
    //all the inputs connected to some action
    //control the opening and closing of the claws
    //driver 2 control
    if(driver2.getRawButton(6)) rightClaw.set(Value.kReverse);
    if(driver2.getRawButton(5)) leftClaw.set(Value.kForward);
    if(driver2.getRawAxis(3) > 0.75) rightClaw.set(Value.kForward);
    if(driver2.getRawAxis(2) > 0.75) leftClaw.set(Value.kReverse);
    //driver 1 control
    if(leftStick.getTriggerPressed()) {
      if(leftClaw.get() == Value.kForward) {
        leftClaw.set(Value.kReverse);
      } else {
        leftClaw.set(Value.kForward);
      }
    }
    if(rightStick.getTriggerPressed()) {
      if(rightClaw.get() == Value.kForward) {
        rightClaw.set(Value.kReverse);
      } else {
        rightClaw.set(Value.kForward);
      }
    }

  if(driver2.getPOV() == 0) {
    drive(0, 0, false, 0);
    armWrist.set(Value.kForward);
    Timer.delay(0.75);
    armMain.set(Value.kReverse);
    Timer.delay(0.4);
    armWrist.set(Value.kReverse);
    /*t1.reset(); t1.start();*/ 
  }
  if(driver2.getPOV() == 180) {
    drive(0, 0, false, 0);
    armWrist.set(Value.kForward);
    Timer.delay(0.4);
    armMain.set(Value.kForward);
    Timer.delay(0.75);
    armWrist.set(Value.kReverse); 
    /*armMain.set(Value.kForward); t2.reset(); t2.start();*/ 
  }
    //manual control of the wrist joint
    if(driver2.getRawButton(4)) { armWrist.set(Value.kForward); }
    if(driver2.getRawButton(1)) { armWrist.set(Value.kReverse); }

    //change the color of the LEDs, between red, blue, rainbow, and off
    if(leftStick.getRawButton(3)) color = Color.red;
    if(leftStick.getRawButton(4)) color = Color.rainbow;
    if(leftStick.getRawButton(5)) color = Color.blue;
    if(leftStick.getRawButton(6)) color = Color.off;
  }

  public void disabledInit() {
    color = Color.off;
    flag.set(Value.kReverse);
  }

  /**Custom method to drive the robot in tank drive mode, with square input mode
   * @param left_speed The speed of the left motors
   * @param right_speed The speed of the right motors
   * @param squareInputs Whether or not to square the inputs before driving(makes it easier to control at lower speeds)
   * @param deadZone how big of a dead zone to have on the joysticks, bigger number means smaller, recommended 20
  */
  private void drive(double left_speed, double right_speed, boolean squareInputs, float deadZone) {
    //look into if you really want to, but I'll warn you; it's pretty complicated
    //(not as complicated as the move method, however);
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

  /**Extension of the move() method, except there is only need to input one distance
   * @param distance The distance for the robot to travel
   * @param speed The speed to move at
   * @param precision The distance to be acceptable to be off by - WARNING - set this value higher than 0.5
   * @return the time (in seconds) to complete the move
   */
  private double moveStraight(double distance, double speed, double precision) {
    return move(distance, distance, speed, precision);
  }

  /**Extension of the move() method, except specifically for rotating a specific number of degrees
   * @param degrees The degree value to rotate
   * @param speed the speed to complete the rotation - higher values will be less accurate
   * @return the time (in seconds) to complete the rotation
   */
  private double rotate(double degrees, double speed) {
    return move(degrees * Math.PI / 180 * 11.375, degrees * Math.PI / -180 * 11.375, speed, 0.5);
  }

  /**A method for moving accurate distances due to the encoders on the SparkMAX motor controllers - not the best method
   * @param leftDistance The distance to move on the left side
   * @param rightDistance The distance to move on the right side
   * @param speed The speed to move the motors
   * @param precision The distance to be acceptable to be off by - WARNING - set this value higher than 0.5
   * @return the time (in seconds) to complete the move
   */
  private double move(double leftDistance, double rightDistance, double speed, double precision) {
    double speed2 = speed;
    Timer timer = new Timer();
    timer.reset();
    timer.start();
    speed = Math.abs(speed);
    precision = Math.abs(precision);
    e1.setPosition(0);
    e2.setPosition(0);
    if(speed > 0.1) {
      speed2 = 0.1;
      while((Math.abs(e1.getPosition() - rightDistance) > 5 || Math.abs(e2.getPosition() - leftDistance) > 5) && auto.get() < 10.0f) {
        if(e1.getPosition() < rightDistance - 4) {
          rightMotor1.set(speed);
          rightMotor2.set(speed);
          rightMotor3.set(speed);
        } else if(e1.getPosition() > rightDistance + 4) {
          rightMotor1.set(-speed);
          rightMotor2.set(-speed);
          rightMotor3.set(-speed);
        } else {
          rightMotor1.set(0);
          rightMotor2.set(0);
          rightMotor3.set(0);
        }
        if(e2.getPosition() < leftDistance - 4) {
          leftMotor1.set(speed);
          leftMotor2.set(speed);
          leftMotor3.set(speed);
        } else if(e1.getPosition() > leftDistance + 4) {
          leftMotor1.set(-speed);
          leftMotor2.set(-speed);
          leftMotor3.set(-speed);
        } else {
          leftMotor1.set(0);
          leftMotor2.set(0);
          leftMotor3.set(0);
        }
      }
    }
    //precision movement
    boolean state1 = true;
    boolean state2 = true;
    while((state1 || state2) && auto.get() < 10.0f) {
      if(e1.getPosition() < rightDistance - precision) {
        state1 = true;
        rightMotor1.set(speed2);
        rightMotor2.set(speed2);
        rightMotor3.set(speed2);
      } else if(e1.getPosition() > rightDistance + precision) {
        state1 = true;
        rightMotor1.set(-speed2);
        rightMotor2.set(-speed2);
        rightMotor3.set(-speed2);
      } else {
        rightMotor1.set(0);
        rightMotor2.set(0);
        rightMotor3.set(0);
        state1 = false;
      }
      if(e2.getPosition() < leftDistance - precision) {
        state2 = true;
        leftMotor1.set(speed2);
        leftMotor2.set(speed2);
        leftMotor3.set(speed2);
      } else if(e1.getPosition() > leftDistance + precision) {
        state2 = true;
        leftMotor1.set(-speed2);
        leftMotor2.set(-speed2);
        leftMotor3.set(-speed2);
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

  /**Custom method to create rainbow effect for LED strip - must be called repeatedly */
  private void rainbow() {
    // For every pixel
    for (var i = 0; i < buffer.getLength(); i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      final var hue = (m_rainbowFirstPixelHue + (i * 360 / buffer.getLength())) % 180;
      // Set the value
      buffer.setHSV(i, (int)hue, 255, 128);
    }
    // Increase by to make the rainbow "move"
    m_rainbowFirstPixelHue += 2;
    // Check bounds
    m_rainbowFirstPixelHue %= 180;
  }
}
