package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shoulder extends SubsystemBase {

    public static final double LOW_TARGET_HEIGHT = 30/360d;
    public static final double HIGH_TARGET_HEIGHT = 100/360d;
    public static final double MID_TARGET_HEIGHT = 90/360d;
    public static final double HUMAN_PLAYER_HEIGHT = 80/360d;
    
    private final PIDController pid;
    private boolean enabled = true;

    private final VictorSPX leftTop = new VictorSPX(12);
    private final VictorSPX leftBottom = new VictorSPX(13);
    
    private final VictorSPX rightTop = new VictorSPX(10);
    private final VictorSPX rightBottom = new VictorSPX(11);

    private final Encoder encoder = new Encoder(6, 7);

    public Shoulder() {
        pid = new PIDController(3.7, 0, 0.4);
        pid.enableContinuousInput(0, 1);
        pid.setSetpoint(0);
        encoder.setDistancePerPulse(1d/600d);
        encoder.reset();
    }

    public void resetPosition() {
        encoder.reset();
    }

    public void update() {
        double position = (-encoder.getDistance()) % 1d;
        position = position < 0 ? position + 1 : position;
        if(position > 0.4 && position < 0.97) {
            enabled = false;
        }
        SmartDashboard.putNumber("position", position);
        SmartDashboard.putBoolean("shoulder enabled", enabled);
        
        if(!enabled) {
            return;
        }

        double movement = pid.calculate(position);

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

    @Override
    public void periodic() {
        update();
    }
}
