package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Telescope extends SubsystemBase {
    VictorSPX motor1 = new VictorSPX(5);
    // VictorSPX motor2 = new VictorSPX(5);

    public void setSpeed(double speed) {
        motor1.set(ControlMode.PercentOutput, speed);
        // motor2.set(ControlMode.PercentOutput, speed);
    }
}
