package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shoulder extends SubsystemBase {
    VictorSPX motor1 = new VictorSPX(1);
    VictorSPX motor2 = new VictorSPX(2);
    VictorSPX motor3 = new VictorSPX(3);
    VictorSPX motor4 = new VictorSPX(4);
    
    double throttle;

    public Shoulder(double throttle) {
        this.throttle = throttle;
    }

    public void setshoulderspeed(double left, double right) {
        motor1.set(ControlMode.PercentOutput, left * throttle);
        motor2.set(ControlMode.PercentOutput, left * throttle);
        motor3.set(ControlMode.PercentOutput, right * throttle);
        motor4.set(ControlMode.PercentOutput, right * throttle);
    }
}
