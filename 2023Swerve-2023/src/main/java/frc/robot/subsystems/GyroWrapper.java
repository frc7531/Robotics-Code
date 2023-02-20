package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;

public class GyroWrapper extends ADXRS450_Gyro {
    private Rotation2d offset;

    public GyroWrapper() {
        super();
        offset = new Rotation2d(0);
    }

    @Override
    public Rotation2d getRotation2d() {
        return super.getRotation2d().minus(offset).plus(Rotation2d.fromDegrees(180));
    }

    public void resetAngle(Rotation2d newAngle) {
        offset = newAngle.minus(super.getRotation2d());
    }
    
    public void resetAngle() {
        offset = super.getRotation2d().times(-1);
    }
}
