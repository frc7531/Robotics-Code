// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CameraSubsystem extends SubsystemBase {
  private static final Transform3d[] absolutePositions = new Transform3d[] {
    null, null, null, null, null, null,
    new Transform3d(new Translation3d(0, 2.494, 0.392), new Rotation3d(0, 0, 0)),
    new Transform3d(new Translation3d(0, 2, 0.385), new Rotation3d(0, 0, 0)),
    null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null, null,
    null, null, null, null, null, null, null
  };

  private static final Transform3d cameraPosition = new Transform3d(new Translation3d(-0.35, 0, -0.17), new Rotation3d(0, 0, 0));

  private final PhotonCamera camera;
  private final SwerveDrive swerve;

  private Transform3d[] transforms = new Transform3d[32];
  private PhotonPipelineResult lastResult;

  public CameraSubsystem(SwerveDrive swerve) {
    this.swerve = swerve;
    camera = new PhotonCamera("camera0");
    camera.setPipelineIndex(0);
  }

  @Override
  public void periodic() {
    update();
    if(transforms[0] != null) {
      SmartDashboard.putString("distanceY", transforms[0].toString());
    }
  }

  public void update() {
    PhotonPipelineResult result = camera.getLatestResult();

    // If the latest result is the same as the previous result, then the camera didn't
    // update yet and the results are not valid anymore.
    if(result.equals(lastResult) || result.targets.isEmpty()) {
      return;
    }

    // Loop through all the found targets and add them into the array of transforms for
    // later use
    for(PhotonTrackedTarget target : result.targets) {
      transforms[target.getFiducialId()] = target.getBestCameraToTarget();
    }

    // Now that we have a list of all the targets, we can take the real positions of the
    // targets as transforms and subtract where the target is relative to the robot, and
    // that will give us the absolute position of the robot relative to the origin of the
    // field.
    // Transform3d totalGuesses = new Transform3d();
    // double totalWeights = 0;

    // for(PhotonTrackedTarget target : result.targets) {
    //   if(absolutePositions[target.getFiducialId()] == null) {
    //     continue;
    //   }
    //   Transform3d guess = absolutePositions[target.getFiducialId()]
    //     .plus(
    //       target.getBestCameraToTarget()
    //         .inverse()
    //     );
    //   // guess = guess.times(1 - target.getPoseAmbiguity());
    //   totalGuesses = totalGuesses.plus(guess);
    //   totalWeights += 1; // - target.getPoseAmbiguity();
    // }

    if(absolutePositions[result.getBestTarget().getFiducialId()] == null ||
      result.getBestTarget().getPoseAmbiguity() > 0.2) {
      return;
    }

    Transform3d masterPosition = // totalGuesses.div(totalWeights);
    absolutePositions[result.getBestTarget().getFiducialId()]
      .plus(
        result.getBestTarget().getBestCameraToTarget()
          .inverse()
      )
      .plus(cameraPosition);

    Pose2d masterPose = new Pose2d(
      masterPosition
        .getTranslation()
        .toTranslation2d(), 
      masterPosition
        .getRotation()
        .toRotation2d()
    );

    masterPose = new Pose2d(masterPose.getTranslation(), Rotation2d.fromRadians(-masterPose.getRotation().getRadians()));

    SwerveDriveOdometry odometer = swerve.getOdometer();

    Transform2d change = masterPose.minus(odometer.getPoseMeters());
    change = change.times(0.4);

    swerve.setOdometer(odometer.getPoseMeters().plus(change), odometer.getPoseMeters().getRotation().plus(change.getRotation()));

    // swerve.getGyro().resetAngle(masterPose.getRotation());
    
    SmartDashboard.putNumber("X", odometer.getPoseMeters().getX());
    SmartDashboard.putNumber("Y", odometer.getPoseMeters().getY());
    SmartDashboard.putNumber("Angle", swerve.getGyro().getRotation2d().getDegrees());
    
    lastResult = result;
  }

  public Transform3d getTagLocation(int id) {
    return transforms[id];
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
