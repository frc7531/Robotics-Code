package frc.robot;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveDrive {
    private MotorGroup[] motorGroups;
    private SwerveDriveKinematics kinematics;
    private int numModules;

    private ADXRS450_Gyro gyro;
    private SwerveDriveOdometry odometer;

    public SwerveDrive(MotorGroup[] motorGroups, SwerveDriveKinematics kinematics, ADXRS450_Gyro gyro) {
        this.kinematics = kinematics;
        this.motorGroups = motorGroups;
        this.numModules = motorGroups.length;
        this.gyro = gyro;
        odometer = new SwerveDriveOdometry(kinematics, gyro.getRotation2d());
    }

    public void drive(ChassisSpeeds speeds, double rotationSpeed, double marginOfError) {
        SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(speeds);
        for(int i = 0; i < numModules; i++) {
            motorGroups[i].getState();
        }
        odometer.update(gyro.getRotation2d(), moduleStates);
        SmartDashboard.putString("Thing", odometer.getPoseMeters().toString());
        System.out.println(odometer.getPoseMeters().toString());
        for(int i = 0; i < numModules; i++) {
            motorGroups[i].setDifferential(moduleStates[i].speedMetersPerSecond / 2, moduleStates[i].angle.getDegrees(), rotationSpeed, marginOfError);
        }
    }

    public SwerveDriveOdometry getOdometer() { return odometer; }
}