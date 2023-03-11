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
    //private final TalonSRX motor2 = new TalonSRX(15);

    private final DigitalInput rearLimit = new DigitalInput(4);

    private double target;

    private final Encoder encoder = new Encoder(8, 9);

    private final PIDController pid = new PIDController(1, 0, 0);

    private final boolean debug = false;

    @Override
    public void periodic() {
        setSpeed(pid.calculate(encoder.getDistance(), target));
        if (!debug) {return;}
        SmartDashboard.putBoolean("rear limit switch", !rearLimit.get());
        SmartDashboard.putNumber("tele-coder", encoder.getDistance());
    }

    public void setSpeed(double speed) {
        if (debug) { 
            SmartDashboard.putNumber("input speed", speed); 
        }

        if (!rearLimit.get() && speed > 0) {
            setMotors(0);
            encoder.reset();
            return;
        }
        
        motor1.set(ControlMode.PercentOutput, -speed);
    }

    public void setPosition(double position) {
        target = position;
    }

    private void setMotors(double speed) {
       // motor2.set(ControlMode.PercentOutput,  speed);
    }
}
