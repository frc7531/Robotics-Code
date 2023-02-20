package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.AbsoluteEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shoulder extends SubsystemBase {
    private final PIDController pid;

    private final VictorSPX leftTop = new VictorSPX(12);
    private final VictorSPX leftBottom = new VictorSPX(13);
    
    private final VictorSPX rightTop = new VictorSPX(10);
    private final VictorSPX rightBottom = new VictorSPX(11);

    private final AnalogEncoder encoder = new AnalogEncoder(0);

    private final double throttle = 0.3;

    private double lastPosition = 0;
    private double lastTime = 0;

    public Shoulder() {
        pid = new PIDController(4, 0, 0.3);
        encoder.reset();
    }

    public void resetPosition() {
        encoder.reset();
    }

    public void setSpeed(double inputSpeed) {
        double target = inputSpeed * throttle;

        double position = -encoder.getDistance();

        SmartDashboard.putNumber("current", position);

        double movement = pid.calculate(position, target);
        SmartDashboard.putNumber("movement", movement);

        leftTop.set(ControlMode.PercentOutput, -movement);
        leftBottom.set(ControlMode.PercentOutput, -movement);

        rightTop.set(ControlMode.PercentOutput, movement);
        rightBottom.set(ControlMode.PercentOutput, movement);

        lastPosition = position;
    }
}
