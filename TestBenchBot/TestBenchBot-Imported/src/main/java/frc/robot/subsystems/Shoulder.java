package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shoulder extends SubsystemBase {
    VictorSPX motor1;
    VictorSPX motor2;

    double throttle;

    public Shoulder(int motor1Port, int motor2Port, double throttle) {
        motor1 = new VictorSPX(motor1Port);
        motor2 = new VictorSPX(motor2Port);
        //motor1.setInverted(true);

        this.throttle = throttle;
    }

    public void setshoulderspeed(double speed) {
        motor1.set(ControlMode.PercentOutput, speed * throttle);
        motor2.set(ControlMode.PercentOutput, speed * throttle);
    }
}
