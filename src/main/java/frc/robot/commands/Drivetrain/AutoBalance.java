// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class AutoBalance extends CommandBase {
  /** Creates a new AutoBalance. */
  boolean end;
  double pitch, kP = 0.3;
  public AutoBalance() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_drivetrain);
    end = false;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pitch = RobotContainer.m_drivetrain.getPitch();
    while (pitch > 2.5 || pitch < -2.5) {
      if (pitch > 0) {
        RobotContainer.m_drivetrain.tankDrive(kP, kP);
      } else if (pitch < 0) {
        RobotContainer.m_drivetrain.tankDrive(-kP, -kP);
      }
    }
    end = true;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return end;
  }
}
