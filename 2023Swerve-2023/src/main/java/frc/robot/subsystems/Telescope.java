package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Telescope extends SubsystemBase {
    private final TalonSRX motor1 = new TalonSRX(14);
    private final TalonSRX motor2 = new TalonSRX(15);

    private final DigitalInput rearLimit = new DigitalInput(4);
    private final DigitalInput frontLimit = new DigitalInput(5);

    private final boolean debug = false;

    @Override
    public void periodic() {
        if (!debug) {return;}
        SmartDashboard.putBoolean("rear limit switch", !rearLimit.get());
        SmartDashboard.putBoolean("front limit switch", !frontLimit.get());
    }

    public void setSpeed(double speed) {
        if (debug) { SmartDashboard.putNumber("input speed", speed); }

        if (!rearLimit.get() && speed > 0) {
            setMotors(0);
            return;
        }
        if (!frontLimit.get() && speed < 0) {
            setMotors(0);
            return;
        }

        setMotors(speed);
    }

    private void setMotors(double speed) {
        motor1.set(ControlMode.PercentOutput,  speed);
        motor2.set(ControlMode.PercentOutput, -speed);
    }
}
