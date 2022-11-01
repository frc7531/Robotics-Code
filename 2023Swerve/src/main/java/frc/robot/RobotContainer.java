// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.Calibrate;
import frc.robot.commands.TeleopDrive;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.SwerveDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SwerveDrive swerve;
  private final ADXRS450_Gyro gyro;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    gyro = new ADXRS450_Gyro();
    swerve = new SwerveDrive(gyro);
    swerve.setDefaultCommand(new TeleopDrive(swerve, gyro));
    // Configure the button bindings
    configureButtonBindings();

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
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in 
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(0.25, 0.25)
      .setKinematics(swerve.getKinematics());

        // 2. Generate trajectory
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
      new Pose2d(0, 0, new Rotation2d(0)),
      List.of(
        new Translation2d(0, 0),
        new Translation2d(0.5, 0)
      ),
      new Pose2d(0.5, 0, Rotation2d.fromDegrees(0)),
      trajectoryConfig);
      
    PIDController xController = new PIDController(0.1, 0, 0);
    PIDController yController = new PIDController(0.1, 0, 0);
    ProfiledPIDController thetaController = new ProfiledPIDController(
      0.05, 0, 0, new TrapezoidProfile.Constraints(0.05, 0.10)
    );

    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    Command swerveCommand = new SwerveControllerCommand(
      trajectory,
      swerve::getPose,
      swerve.getKinematics(),
      xController,
      yController,
      thetaController,
      swerve::setModuleStates,
      swerve
    );

    return new SequentialCommandGroup(
      new InstantCommand(() -> swerve.resetOdometer()),
      swerveCommand
    );
  }
}