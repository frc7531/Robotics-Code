package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class MotorGroup {
    public CANSparkMax motor1;
    public CANSparkMax motor2;
    public DutyCycleEncoder encoder;
    public PIDController movePidController;
    public PIDController turnPidController;

    public RelativeEncoder enc1;
    public RelativeEncoder enc2;

    private static double conversionFactor = (Math.PI * 8) / (double)35433;

    public MotorGroup(CANSparkMax motor1, CANSparkMax motor2, DutyCycleEncoder encoder) {
        this.motor1 = motor1;
        this.motor2 = motor2;
        motor1.setSmartCurrentLimit(40);
        motor2.setSmartCurrentLimit(40);
        this.encoder = encoder;
        enc1 = motor1.getEncoder();
        enc2 = motor2.getEncoder();
        enc1.setVelocityConversionFactor(conversionFactor);
        enc2.setVelocityConversionFactor(conversionFactor);
        enc1.setPositionConversionFactor(conversionFactor * 60);
        enc2.setPositionConversionFactor(conversionFactor * 60);
        movePidController = new PIDController(0.1, 0.5, 0);
        turnPidController = new PIDController(-0.002, 0, 0);
        turnPidController.setTolerance(0.1);
        turnPidController.enableContinuousInput(0, 360);
    }

    public void setDifferential(double speed, double angle, double rotationSpeed, double marginOfError) {
        // double turnSpeed = this.setAngle(angle, rotationSpeed, marginOfError);
        double turnSpeed = turnPidController.calculate(getAngle(), angle);
        speed = movePidController.calculate(this.getDrivingVelocity(), speed);
        motor1.set(turnSpeed + speed);
        motor2.set(turnSpeed - speed);
    }

    // public void setNonDifferential(double speed, double angle, double rotationSpeed, double marginOfError) {
    //     double turnSpeed = this.setAngle(angle, rotationSpeed, marginOfError);
    //     motor1.set(speed);
    //     motor2.set(turnSpeed);
    // }

    public double getAngle() {
        return (encoder.getAbsolutePosition() * 360);
    }

    private double setAngle(double target, double speed, double marginOfError) {
        target /= 360;
        double current = this.getAngle() / 360;
        /*double calculated = pid.calculate(current, target);
        System.out.println("Current: " + current + ", Target: " + target + ", Calculated: " + calculated);
        if(calculated > 1) calculated = 1;
        if(calculated < -1) calculated = -1;
        return calculated;*/
        double temp1 = current - target;
        double temp2 = current - target - 1.0;
        double distance = Math.min(Math.abs(temp1), Math.abs(temp2));
        double temp3 = 0;
        if(distance == Math.abs(temp1)) {
            temp3 = temp1;
        } else if(distance == Math.abs(temp2)) {
            temp3 = temp2;
        }
        if(Math.abs(temp3) > marginOfError) {
            return speed * temp3;
        } else {
            return 0;
        }
    }

    public double getDrivingVelocity() {
        double speed1 = enc1.getVelocity();
        double speed2 = enc2.getVelocity();
        return speed1 - speed2;
    }

    public double getSteeringVelocity() {
        double speed1 = enc1.getVelocity();
        double speed2 = enc2.getVelocity();
        return (speed1 + speed2) / 2;
    }

    public double getDrivenDistance() {
        double dist1 = enc1.getPosition();
        double dist2 = enc2.getPosition();
        return (dist1 - dist2);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDrivingVelocity(), Rotation2d.fromDegrees(getAngle()));
    }

    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(getDrivenDistance(), Rotation2d.fromDegrees(-getAngle()));
    }
}