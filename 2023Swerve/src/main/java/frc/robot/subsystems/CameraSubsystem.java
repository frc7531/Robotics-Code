// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import javax.swing.TransferHandler.TransferSupport;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CameraSubsystem extends SubsystemBase {
  private PhotonCamera camera;

  private Transform3d[] transforms = new Transform3d[16];
  private double[] reliability = new double[16];

  /** Creates a new ExampleSubsystem. */
  public CameraSubsystem() {
    camera = new PhotonCamera("camera");
    camera.setPipelineIndex(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    update();
    if(transforms[0] != null) {
      SmartDashboard.putString("distanceY", transforms[0].toString());
    }
  }

  public void update() {
    PhotonPipelineResult result = camera.getLatestResult();
    List<PhotonTrackedTarget> targets = result.targets;

    for(int i = 0; i < reliability.length; i++) {
      reliability[i] *= 0.8;
    }

    for(PhotonTrackedTarget target : targets) {
      if(target == null) {
        continue;
      }

      transforms[target.getFiducialId()] = target.getBestCameraToTarget();
      reliability[target.getFiducialId()] = 1;
    }
  }

  public Transform3d getTagLocation(int id) {
    return transforms[id];
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
