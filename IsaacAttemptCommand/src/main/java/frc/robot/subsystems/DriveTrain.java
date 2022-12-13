package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class DriveTrain extends SubsystemBase{
    private TalonSRX motor1;
    private TalonSRX motor2;
    private TalonSRX motor3;
    private TalonSRX motor4;
    private TalonSRX motor5;
    private TalonSRX motor6;


    public DriveTrain(){
        motor1 = new TalonSRX(0);
        motor2 = new TalonSRX(1);
        motor3 = new TalonSRX(2);
        motor4 = new TalonSRX(3);
        motor5 = new TalonSRX(4);
        motor6 = new TalonSRX(5);
        motor1.setInverted(true);
        motor2.setInverted(true);
        motor3.setInverted(true);
       
    }

    public void Left(double leftJoystick){
        motor1.set(ControlMode.PercentOutput, leftJoystick);
        motor2.set(ControlMode.PercentOutput, leftJoystick);
        motor3.set(ControlMode.PercentOutput, leftJoystick);
      
    }
    public void Right(double rightJoystick){
        motor4.set(ControlMode.PercentOutput, rightJoystick);
        motor5.set(ControlMode.PercentOutput, rightJoystick);
        motor6.set(ControlMode.PercentOutput, rightJoystick);
      
    }
    public void telopDrive(double leftJoystick, double rightJoystick){
        Right(rightJoystick);
        Left(leftJoystick);
    }
    
}
