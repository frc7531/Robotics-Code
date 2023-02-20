package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class DriveTrain extends SubsystemBase{
    private TalonSRX leftMotor1;
    private TalonSRX leftMotor2;

    private TalonSRX rightMotor1;
    private TalonSRX rightMotor2;

    public DriveTrain() {
        leftMotor1 = new TalonSRX(1);
        leftMotor2 = new TalonSRX(2);
        rightMotor1 = new TalonSRX(3);
        rightMotor2 = new TalonSRX(5);
    }

    public void tankDrive(double leftSpeed, double rightSpeed) {
        leftMotor1.set(ControlMode.PercentOutput, -leftSpeed);
        leftMotor2.set(ControlMode.PercentOutput, -leftSpeed);
        rightMotor1.set(ControlMode.PercentOutput, rightSpeed);
        rightMotor2.set(ControlMode.PercentOutput, rightSpeed);
    }

    public void arcadeDrive(double moveSpeed, double turnSpeed) {
        leftMotor1.set(ControlMode.PercentOutput, -(moveSpeed - turnSpeed));
        leftMotor2.set(ControlMode.PercentOutput, -(moveSpeed - turnSpeed));
        rightMotor1.set(ControlMode.PercentOutput, moveSpeed + turnSpeed);
        rightMotor2.set(ControlMode.PercentOutput, moveSpeed + turnSpeed);
    }
}
