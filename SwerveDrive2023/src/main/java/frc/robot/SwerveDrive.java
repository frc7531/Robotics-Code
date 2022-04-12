package frc.robot;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveDrive {
    private MotorGroup[] motorGroups;
    private SwerveDriveKinematics kinematics;
    private int numModules;

    public SwerveDrive(MotorGroup[] motorGroups, SwerveDriveKinematics kinematics) {
        this.kinematics = kinematics;
        this.motorGroups = motorGroups;
        this.numModules = motorGroups.length;
    }

    public void drive(ChassisSpeeds speeds, double rotationSpeed, double marginOfError) {
        SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(speeds);
        for(int i = 0; i < numModules; i++) {
            motorGroups[i].setDifferential(moduleStates[i].speedMetersPerSecond, moduleStates[i].angle.getDegrees(), rotationSpeed, marginOfError);
        }
    }
}