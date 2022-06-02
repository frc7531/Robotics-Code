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
        return encoder.getAbsolutePosition();
    }

    private double setAngle(double target, double speed, double marginOfError) {
        target /= 360;
        double current = this.getAngle();
        double temp1 = current - target;
        double temp2 = current - target - 1.0;
        double distance = Math.min(Math.abs(temp1), Math.abs(temp2));
        double temp3 = 0.2;
        if(distance == Math.abs(temp1)) {
            temp3 = temp1;
        } else if(distance == Math.abs(temp2)) {
            temp3 = temp2;
        }
        if(Math.abs(temp3) > marginOfError) {
            return speed * temp3 + 0.1;
        } else {
            System.out.println("inside margin");
            return 0;
        }
    }
}