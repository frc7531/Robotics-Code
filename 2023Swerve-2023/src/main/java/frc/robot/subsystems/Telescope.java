package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Telescope extends SubsystemBase {
    private final TalonSRX motor1 = new TalonSRX(14);
    private final DigitalInput rearLimit = new DigitalInput(4);
    private final Encoder encoder = new Encoder(8, 9);
    private final PIDController pid = new PIDController(0.01, 0, 0);

    private double target;

    public Telescope() {
        pid.setTolerance(100);
    }

    @Override
    public void periodic() {
        double calculation = pid.calculate(encoder.getDistance(), target);
        setSpeed(calculation);
        SmartDashboard.putNumber("tele-target", target);
        SmartDashboard.putNumber("force", calculation);
        SmartDashboard.putBoolean("rear limit switch", !rearLimit.get());
        SmartDashboard.putNumber("tele-coder", encoder.getDistance());
    }
    
    public void setSpeed(double speed) {
        if (!rearLimit.get() && speed > 0) {
            encoder.reset();
            if(Math.abs(target - 2000) < 200) target = 0;
            motor1.set(ControlMode.PercentOutput, 0);
        } else {
            motor1.set(ControlMode.PercentOutput, -speed);
        }
    }

    public void setPosition(double position) {
        target = position;
    }

    public boolean atSetpoint() {
        return pid.atSetpoint();
    }

    public void reset() {
        target = 2000;
    }
}
