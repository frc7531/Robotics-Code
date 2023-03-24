package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shoulder extends SubsystemBase {    
    private final PIDController pid;

    private final VictorSPX leftTop = new VictorSPX(12);
    private final VictorSPX leftBottom = new VictorSPX(13);
    
    private final VictorSPX rightTop = new VictorSPX(10);
    private final VictorSPX rightBottom = new VictorSPX(11);

    // private final Encoder encoder = new Encoder(6, 7);
    private final AnalogEncoder encoder = new AnalogEncoder(0);

    public Shoulder() {
        pid = new PIDController(3.7, 0, 0.4);
        pid.enableContinuousInput(0, 1);
        pid.setSetpoint(0);
        pid.setTolerance(0.08);
        // encoder.setDistancePerPulse(1d/600d);
        encoder.reset();
    }

    public void resetPosition() {
        encoder.reset();
    }

    public void update() {
        double position = (encoder.getDistance()) % 1d;
        position = position < 0 ? position + 1 : position;

        double movement = pid.calculate(position);

        setMotors(movement);
    }

    private void setMotors(double movement) {

        leftTop.set(ControlMode.PercentOutput, movement);
        leftBottom.set(ControlMode.PercentOutput, movement);

        rightTop.set(ControlMode.PercentOutput, -movement);
        rightBottom.set(ControlMode.PercentOutput, -movement);

    }

    public void setHeight(double target) {
        pid.setSetpoint(target);
    }

    public double getHeight() {
        return pid.getSetpoint();
    }

    public boolean atSetpoint() {
        return pid.atSetpoint();
    }

    @Override
    public void periodic() {
        update();
    }
}
