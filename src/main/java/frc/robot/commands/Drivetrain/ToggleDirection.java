// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class ToggleDirection extends CommandBase {
  /** Creates a new ToggleDirection. */
  boolean end;
  public ToggleDirection() {
    // Use addRequirements() here to declare subsystem dependencies.
    end = false;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (DriveTrain.isReversed == false) {
      DriveTrain.isReversed = true;
    } else if (DriveTrain.isReversed == true) {
      DriveTrain.isReversed = false;
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
