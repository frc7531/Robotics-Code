package frc.robot;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class MotorGroup {
    public CANSparkMax motor1;
    public CANSparkMax motor2;
    public DutyCycleEncoder encoder;

    public MotorGroup(CANSparkMax motor1, CANSparkMax motor2, DutyCycleEncoder encoder) {
        this.motor1 = motor1;
        this.motor2 = motor2;
        this.encoder = encoder;
    }

    public void setDifferential(double speed, double angle, double rotationSpeed, double marginOfError) {
        double turnSpeed = this.setAngle(angle, rotationSpeed, marginOfError);
        motor1.set(turnSpeed + speed);
        motor2.set(turnSpeed - speed);
    }

    public void setNonDifferential(double speed, double angle, double rotationSpeed, double marginOfError) {
        double turnSpeed = this.setAngle(angle, rotationSpeed, marginOfError);
        motor1.set(speed);
        motor2.set(turnSpeed);
    }

    public double getAngle() {
        if(encoder.get() < 0) {
            return (encoder.get() % 1.0 + 1) * 360;
        } else {
            return (encoder.get() % 1.0) * 360;
        }
    }

    private double setAngle(double target, double speed, double marginOfError) {
        double current = this.getAngle();
        double temp1 = target - current;
        double temp2 = target - current - 360;
        double distance = Math.min(Math.abs(temp1), Math.abs(temp2));
        if(distance < marginOfError && distance > -5) {
            return 0;
        } else {
            return Math.copySign(speed, distance);
        }
    }
}
