// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class GetOnRamp extends CommandBase {
  /** Creates a new getOnRamp. */
  boolean end;
  public GetOnRamp() {
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
    System.out.println("getting on ramp...");
    double pitch = Math.abs(RobotContainer.m_drivetrain.getPitch());
    if (pitch != Constants.AutoConstants.onRampGyro) {
      RobotContainer.m_drivetrain.tankDrive(0.5, 0.5);
    }

    if (pitch < Constants.AutoConstants.onRampGyro + 0.25 && pitch > Constants.AutoConstants.onRampGyro - 0.25) {
      RobotContainer.m_drivetrain.tankDrive(0, 0);
      RobotContainer.m_drivetrain.setBreakMode();
      end = true;
    }
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
