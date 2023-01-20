// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.net.URI;
import java.nio.file.Path;
import java.util.List;

import javax.tools.StandardJavaFileManager.PathFactory;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.spline.Spline;
import edu.wpi.first.math.spline.Spline.ControlVector;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrajectoryGenerator.ControlVectorList;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.Calibrate;
import frc.robot.commands.PointToTarget;
import frc.robot.commands.TeleopDrive;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.SwerveDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final SwerveDrive swerve;

  private final CameraSubsystem cameraSystem;
  
  private final Joystick gXbox;
  private final Joystick rXbox;

  private final ADXRS450_Gyro gyro;
  
  public RobotContainer() {
    gXbox = new Joystick(0);
    rXbox = new Joystick(1);  

    gyro = new ADXRS450_Gyro();

    swerve = new SwerveDrive(gyro);

    cameraSystem = new CameraSubsystem(swerve);
    // cameraSystem.setDefaultCommand(new PointToTarget(cameraSystem, swerve));

    swerve.setDefaultCommand(new TeleopDrive(swerve, gyro, gXbox));
    // swerve.setDefaultCommand(new Calibrate(swerve));
    // Configure the button bindings
    configureButtonBindings();

    //get joystick buttons
    JoystickButton yButton = new JoystickButton(gXbox, 4);

    yButton.onTrue(
      new InstantCommand(() -> swerve.resetGyro())
    );

    // SmartDashboard.putData(new Calibrate(swerve));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   * @throws IOException
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in 
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(2, 0.5)
      .setKinematics(swerve.getKinematics());

        // 2. Generate trajectory
    Trajectory trajectory;

    trajectory = TrajectoryGenerator.generateTrajectory(
      List.of(
        new Pose2d(0, 0, new Rotation2d(Math.PI / 2)),
        new Pose2d(0, 0.5, new Rotation2d(Math.PI / 2)),
        new Pose2d(0, 1, new Rotation2d(Math.PI / 2))
      ),
      trajectoryConfig);

    Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve("paths/Unnamed.wpilib.json");

    try {
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException e) {
      e.printStackTrace();
    }

    PIDController xController = new PIDController(0.4, 0, 0);
    PIDController yController = new PIDController(0.4, 0, 0);
    ProfiledPIDController thetaController = new ProfiledPIDController(
      1, 0, 0, new TrapezoidProfile.Constraints(3, 2)
    );
    thetaController.setTolerance(0.001);
    xController.setTolerance(0.001);
    yController.setTolerance(0.001);

    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    Command swerveCommand = new SwerveControllerCommand(
      trajectory,
      swerve::getPose,
      swerve.getKinematics(),
      xController,
      yController,
      thetaController,
      this::updateTheta,
      swerve::setModuleStates,
      swerve
    );
    Command swerveCommand2 = new SwerveControllerCommand(
      trajectory,
      swerve::getPose,
      swerve.getKinematics(),
      xController,
      yController,
      thetaController,
      this::updateTheta,
      swerve::setModuleStates,
      swerve
    );
    
    return new SequentialCommandGroup(
      new InstantCommand(() -> SmartDashboard.putString("status", "Running")),
      new InstantCommand(() -> swerve.setOdometer(new Pose2d(2, 2, Rotation2d.fromDegrees(0)))),
      swerveCommand,
      swerveCommand2,
      new InstantCommand(() -> SmartDashboard.putString("status", "Done")),
      new InstantCommand(() -> swerve.stopMotors())
    );
  }

  Rotation2d updateTheta() {
    //return new Rotation2d(classic.getRawAxis(2), classic.getRawAxis(3));
    return new Rotation2d(0);
  }
}