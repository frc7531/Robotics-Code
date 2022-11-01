package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDrive extends SubsystemBase {
    private final CANSparkMax[] motors;
    private final DutyCycleEncoder[] encoders;
    private final SwerveDriveKinematics kinematics;
    private MotorGroup[] motorGroups;
    private int numModules;
    private SwerveModuleState[] states;

    private ADXRS450_Gyro gyro;
    private SwerveDriveOdometry odometer;

    public SwerveDrive(ADXRS450_Gyro gyro) {
        Translation2d[] translations = new Translation2d[] {
            new Translation2d(0.5, 0.5),
            new Translation2d(0.5, -0.5),
            new Translation2d(-0.5, -0.5),
            new Translation2d(-0.5, 0.5),
        };
        motors = new CANSparkMax[] {
            new CANSparkMax(1, MotorType.kBrushless),
            new CANSparkMax(2, MotorType.kBrushless),
            new CANSparkMax(3, MotorType.kBrushless),
            new CANSparkMax(4, MotorType.kBrushless),
            new CANSparkMax(5, MotorType.kBrushless),
            new CANSparkMax(6, MotorType.kBrushless),
            new CANSparkMax(7, MotorType.kBrushless),
            new CANSparkMax(8, MotorType.kBrushless),
        };
        encoders = new DutyCycleEncoder[] {
            new DutyCycleEncoder(0),
            new DutyCycleEncoder(1),
            new DutyCycleEncoder(2),
            new DutyCycleEncoder(3),
        };
        motorGroups = new MotorGroup[4];
        for(int i = 0; i < motorGroups.length; i++) {
            motorGroups[i] = new MotorGroup(motors[2*i], motors[2*i + 1], encoders[i]);
        }
        kinematics = new SwerveDriveKinematics(translations);
        this.numModules = motorGroups.length;
        this.gyro = gyro;
        odometer = new SwerveDriveOdometry(kinematics, gyro.getRotation2d());
    }

    public void drive(ChassisSpeeds speeds, double rotationSpeed, double marginOfError) {
        states = kinematics.toSwerveModuleStates(speeds);
        applyStates(rotationSpeed, marginOfError);
    }

    public void applyStates(double rotationSpeed, double marginOfError) {
        SwerveModuleState[] realStates = new SwerveModuleState[4];
        for(int i = 0; i < numModules; i++) {
            realStates[i] = motorGroups[i].getState();
        }
        odometer.update(gyro.getRotation2d(), states);
        SmartDashboard.putNumber("X", odometer.getPoseMeters().getX());
        SmartDashboard.putNumber("Y", odometer.getPoseMeters().getY());
        System.out.println(odometer.getPoseMeters().toString());
        for(int i = 0; i < numModules; i++) {
            motorGroups[i].setDifferential(states[i].speedMetersPerSecond, states[i].angle.getDegrees(), rotationSpeed, marginOfError);
        }
    }

    public void applyStates() {
        applyStates(1, 0);
    }

    public void setModuleStates(SwerveModuleState[] states) {
        this.states = states;
        applyStates();
    }

    public SwerveDriveOdometry getOdometer() { return odometer; }

    public MotorGroup[] getMotorGroups() { return motorGroups; }

    public SwerveDriveKinematics getKinematics() { return kinematics; }

    public Pose2d getPose() { return odometer.getPoseMeters(); }
}