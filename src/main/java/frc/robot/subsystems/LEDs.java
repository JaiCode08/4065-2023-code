// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LEDs extends SubsystemBase {
  /** Creates a new LEDs. */
  public static boolean gameObject; // true --> cube, false --> cone
  private static Spark m_blinkin = null;
  public LEDs() {
    m_blinkin = new Spark(Constants.Other.blinkInPWM);
    gameObject = true;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Game Object", gameObject);
    if (gameObject == true) {
      m_blinkin.set(0.91); // purplse
    } else {
      m_blinkin.set(0.69); // yellow
    }
  }
}
