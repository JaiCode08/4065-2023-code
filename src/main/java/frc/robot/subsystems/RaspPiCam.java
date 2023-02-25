// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RaspPiCam extends SubsystemBase {
  public static PhotonCamera cam = new PhotonCamera("IMX219");
  /** Creates a new RaspPiCam. */
  public RaspPiCam() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public boolean hasTargets() {
    return cam.getLatestResult().hasTargets();
  }

  public double getYaw(double valToReturn) {
    try {
      return (cam.getLatestResult().getBestTarget().getYaw());
    } catch (Exception e) {
      return valToReturn;
    }
  }
}
