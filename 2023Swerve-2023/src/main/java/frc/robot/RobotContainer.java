// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.util.HashMap;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.autoroutines.AutoRoutine;
import frc.robot.commands.Calibrate;
import frc.robot.commands.DumbTelescope;
import frc.robot.commands.TeleopDrive;
import frc.robot.commands.UpdateArmPID;
// import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Telescope;
import frc.robot.subsystems.GyroWrapper;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Shoulder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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

  private final LEDs leds;

  // private final CameraSubsystem cameraSystem;
  
  private final Joystick gXbox;
  private final Joystick rXbox;

  private final Gyro gyro;
  private final Claw claw;
  
  private final JoystickButton clawButton;
  private final JoystickButton coneButton;
  private final JoystickButton cubeButton;

  private final JoystickButton resetGyroButton;

  private final SendableChooser<Command> teleopChooser;
  private final SendableChooser<String> autoChooser;
  
  public RobotContainer() {
    CameraServer.startAutomaticCapture();

    leds = new LEDs();
    leds.setGamerMode(true);

    gXbox = new Joystick(0);
    rXbox = new Joystick(1);

    gyro = new ADXRS450_Gyro();

    swerve = new SwerveDrive(gyro);
    shoulder = new Shoulder();
    telescope = new Telescope();
    
    //get joystick buttons
    clawButton = new JoystickButton(rXbox, 6);

    coneButton = new JoystickButton(gXbox, 5);
    cubeButton = new JoystickButton(gXbox, 6);

    resetGyroButton = new JoystickButton(rXbox, 8);

    claw = new Claw(gXbox);
    shoulder.setDefaultCommand(new UpdateArmPID(shoulder, gXbox));
    telescope.setDefaultCommand(new DumbTelescope(telescope));

    coneButton.onTrue(new InstantCommand(() -> leds.setFullStripColorRGB(128, 64, 0)));
    cubeButton.onTrue(new InstantCommand(() -> leds.setFullStripColor(Color.kPurple)));

    // cameraSystem = new CameraSubsystem(swerve);
    // cameraSystem.setDefaultCommand(new PointToTarget(cameraSystem, swerve));

    // Configure the button bindings
    configureButtonBindings();


    clawButton.onTrue(new InstantCommand(() -> claw.crab()));
    // resetArmEncoderButton.onTrue(new InstantCommand(() -> shoulder.resetPosition()));

    resetGyroButton.whileTrue(
      new InstantCommand(() -> gyro.reset())
    );

    teleopChooser = new SendableChooser<>();
    swerve.setDefaultCommand(new TeleopDrive(swerve, gyro, rXbox));
    teleopChooser.setDefaultOption("Drive", new TeleopDrive(swerve, gyro, rXbox));
    teleopChooser.addOption("Calibrate", new Calibrate(swerve));
    teleopChooser.addOption("None", null);
    SmartDashboard.putData("Drive Mode", teleopChooser);

    autoChooser = new SendableChooser<>();
    autoChooser.setDefaultOption("Test Circle", "Test Circle nospin");
    autoChooser.addOption("None", null);
    SmartDashboard.putData("Autonomous", autoChooser);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}

  public void startTeleOp() {
    System.out.println(teleopChooser.getSelected());
    swerve.setDefaultCommand(teleopChooser.getSelected());
    leds.setGamerMode(false);
    leds.setFullStripColor(Color.kBlack);
  }

  public void endTeleop() {
    leds.setGamerMode(true);
  }

  public void periodic() {
    // SmartDashboard.putNumber("Angle", gyro.getRotation2d().getDegrees());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *$
   * @return the command to run in autonomous
   * @throws IOException
   */
  public Command getAutonomousCommand() {
    // return null;
    HashMap<String, Command> eventMap = new HashMap<>();

    eventMap.put("Raise Shoulder High Target", new InstantCommand(() -> {
      shoulder.setHeight(Shoulder.HIGH_TARGET_HEIGHT);
    }, shoulder));

    return AutoRoutine.getSwerveCommand(swerve, "Test Circle", eventMap);
  //   Trajectory trajectory;

  //   Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve("paths/BasicTest.wpilib.json");

  //   try {
  //     trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
  //   } catch (IOException e) {
  //     e.printStackTrace();
  //     return new InstantCommand(() -> {});
  //   }

  //   PIDController xController = new PIDController(0.4, 0, 0);
  //   PIDController yController = new PIDController(0.4, 0, 0);
  //   ProfiledPIDController thetaController = new ProfiledPIDController(
  //     1, 0, 0, new TrapezoidProfile.Constraints(1, 0.5)
  //   );

  //   thetaController.setTolerance(0.001);
  //   xController.setTolerance(0.001);
  //   yController.setTolerance(0.001);

  //   thetaController.enableContinuousInput(-Math.PI, Math.PI);

  //   Command swerveCommand = new SwerveControllerCommand(
  //     trajectory,
  //     swerve::getPose,
  //     swerve.getKinematics(),
  //     xController,
  //     yController,
  //     thetaController,
  //     this::updateTheta,
  //     swerve::setModuleStates,
  //     swerve
  //   );
    
  //   return new SequentialCommandGroup(
  //     new InstantCommand(() -> swerve.setOdometer(new Pose2d(2, 2, Rotation2d.fromDegrees(0)))),
  //     new InstantCommand(() -> SmartDashboard.putString("status", "Running")),
  //     // swerveCommand,
  //     new InstantCommand(() -> SmartDashboard.putString("status", "Done")),
  //     new InstantCommand(() -> swerve.stopMotors())
  //   );
  }

  // Rotation2d updateTheta() {
  //   //return new Rotation2d(classic.getRawAxis(2), classic.getRawAxis(3));
  //   return new Rotation2d(0);
  // }
}