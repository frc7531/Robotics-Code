package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDrive extends SubsystemBase {
    private static final CANSparkMax[] motors;
    private static final DutyCycleEncoder[] encoders;
    private final SwerveDriveKinematics kinematics;
    private MotorGroup[] motorGroups;
    private int numModules;
    private SwerveModuleState[] states = {
        new SwerveModuleState(0, Rotation2d.fromDegrees(0)),
        new SwerveModuleState(0, Rotation2d.fromDegrees(0)),
        new SwerveModuleState(0, Rotation2d.fromDegrees(0)),
        new SwerveModuleState(0, Rotation2d.fromDegrees(0))
    };
    private SwerveModulePosition[] poses;

    private Gyro gyro;
    private SwerveDriveOdometry odometer;

    static {
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
    }

    public SwerveDrive(Gyro gyro) {
        Translation2d[] translations = new Translation2d[] {
            new Translation2d(-0.5, -0.5),
            new Translation2d(0.5, -0.5),
            new Translation2d(0.5, 0.5),
            new Translation2d(-0.5, 0.5),
        };
        
        motorGroups = new MotorGroup[4];
        poses = new SwerveModulePosition[4];
        for(int i = 0; i < motorGroups.length; i++) {
            motorGroups[i] = new MotorGroup(motors[2*i], motors[2*i + 1], encoders[i]);
            poses[i] = motorGroups[i].getModulePosition();
        }
        kinematics = new SwerveDriveKinematics(translations);
        
        this.numModules = motorGroups.length;
        this.gyro = gyro;
        odometer = new SwerveDriveOdometry(kinematics, gyro.getRotation2d(), poses);
    }

    public void drive(ChassisSpeeds speeds) {
        drive(speeds, 1, 0);
    }

    public void drive(ChassisSpeeds speeds, double rotationSpeed, double marginOfError) {
        SmartDashboard.putNumber("speedX", speeds.vxMetersPerSecond);
        SmartDashboard.putNumber("speedY", speeds.vyMetersPerSecond);
        // speeds.vxMetersPerSecond *= -1;
        states = kinematics.toSwerveModuleStates(speeds);
    }

    public void applyStates(double rotationSpeed, double marginOfError) {
        for(int i = 0; i < numModules; i++) {
            poses[i] = motorGroups[i].getModulePosition();
        }
        odometer.update(gyro.getRotation2d(), poses);
        SmartDashboard.putNumber("X", odometer.getPoseMeters().getX());
        SmartDashboard.putNumber("Y", odometer.getPoseMeters().getY());
        SmartDashboard.putNumber("Angle", odometer.getPoseMeters().getRotation().getDegrees());
        for(int i = 0; i < numModules; i++) {
            motorGroups[i].setDifferential(states[i].speedMetersPerSecond, states[i].angle.getDegrees(), rotationSpeed, marginOfError);
        }
    }

    public void applyStates() {
        applyStates(1, 0);
    }

    public void setModuleStates(SwerveModuleState[] states) {
        this.states = states;
    }

    public void stopMotors() {
        SwerveModuleState[] newStates = kinematics.toSwerveModuleStates(new ChassisSpeeds(0, 0, 0));
        setModuleStates(newStates);
    }

    @Override
    public void periodic() {
        applyStates();
    }

    public SwerveDriveOdometry getOdometer() { return odometer; }

    public MotorGroup[] getMotorGroups() { return motorGroups; }

    public SwerveDriveKinematics getKinematics() { return kinematics; }

    public Pose2d getPose() { return odometer.getPoseMeters(); }

    public void resetOdometer() {
        odometer.resetPosition(new Rotation2d(0), poses, new Pose2d());
    }
    public void setOdometer(Pose2d starting) {
        odometer.resetPosition(new Rotation2d(0), poses, starting);
    }
    public void setOdometer(Pose2d pose, Rotation2d rotation) {
        odometer.resetPosition(rotation, poses, pose);
    }
    public void resetGyro() {
        gyro.reset();
    }
    public Gyro getGyro() {
        return gyro;
    }
}