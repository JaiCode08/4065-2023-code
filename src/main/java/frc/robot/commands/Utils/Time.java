// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Utils;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class Time extends CommandBase {
  /** Creates a new Time. */
  int time_amount;
  boolean end = false;
  public Time(int time_in_ms) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.time_amount = time_in_ms;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    try {
      Thread.sleep(time_amount);
    } catch (InterruptedException e) {
      
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