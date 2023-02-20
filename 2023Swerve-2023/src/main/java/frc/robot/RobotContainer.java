// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.Calibrate;
import frc.robot.commands.DumbShoulder;
import frc.robot.commands.DumbTelescope;
import frc.robot.commands.TeleopDrive;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Telescope;
import frc.robot.subsystems.GyroWrapper;
import frc.robot.subsystems.Shoulder;
import edu.wpi.first.wpilibj2.command.Command;
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
  private final Shoulder shoulder;
  private final Telescope telescope;

  // private final CameraSubsystem cameraSystem;
  
  private final Joystick gXbox;
  private final Joystick rXbox;

  private final GyroWrapper gyro;
  private final Claw claw;
  
  private final JoystickButton clawButton;
  private final JoystickButton resetArmEncoderButton;

  private final SendableChooser<Command> teleopChooser;
  
  public RobotContainer() {
    gXbox = new Joystick(0);
    rXbox = new Joystick(1);

    gyro = new GyroWrapper();

    swerve = new SwerveDrive(gyro);
    shoulder = new Shoulder();
    telescope = new Telescope();

    claw = new Claw(gXbox);
    shoulder.setDefaultCommand(new DumbShoulder(shoulder));
    telescope.setDefaultCommand(new DumbTelescope(telescope));

    // cameraSystem = new CameraSubsystem(swerve);
    // cameraSystem.setDefaultCommand(new PointToTarget(cameraSystem, swerve));

    // Configure the button bindings
    configureButtonBindings();
    
    //get joystick buttons
    clawButton = new JoystickButton(gXbox, 6);
    resetArmEncoderButton = new JoystickButton(gXbox, 4);

    clawButton.onTrue(new InstantCommand(() -> claw.crab()));
    resetArmEncoderButton.onTrue(new InstantCommand(() -> shoulder.resetPosition()));

    // yButton.whileTrue(
    //   new InstantCommand(() -> gyro.resetAngle(gyro.getRotation2d().plus(Rotation2d.fromDegrees(180))))
    // );

    // SmartDashboard.putData(new Calibrate(swerve));

    teleopChooser = new SendableChooser<>();
    teleopChooser.setDefaultOption("Drive", new TeleopDrive(swerve, gyro, gXbox));
    teleopChooser.addOption("Calibrate", new Calibrate(swerve));
    teleopChooser.addOption("None", null);
    SmartDashboard.putData("Drive Mode", teleopChooser);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}

  public void startTeleOp() {
    swerve.setDefaultCommand(teleopChooser.getSelected());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   * @throws IOException
   */
  public Command getAutonomousCommand() {
    Trajectory trajectory;

    Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve("paths/BasicTest.wpilib.json");

    try {
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException e) {
      e.printStackTrace();
      return new InstantCommand(() -> {});
    }

    PIDController xController = new PIDController(0.4, 0, 0);
    PIDController yController = new PIDController(0.4, 0, 0);
    ProfiledPIDController thetaController = new ProfiledPIDController(
      1, 0, 0, new TrapezoidProfile.Constraints(1, 0.5)
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
    
    return new SequentialCommandGroup(
      new InstantCommand(() -> swerve.setOdometer(new Pose2d(2, 2, Rotation2d.fromDegrees(0)))),
      new InstantCommand(() -> SmartDashboard.putString("status", "Running")),
      swerveCommand,
      new InstantCommand(() -> SmartDashboard.putString("status", "Done")),
      new InstantCommand(() -> swerve.stopMotors())
    );
  }

  Rotation2d updateTheta() {
    //return new Rotation2d(classic.getRawAxis(2), classic.getRawAxis(3));
    return new Rotation2d(0);
  }
}