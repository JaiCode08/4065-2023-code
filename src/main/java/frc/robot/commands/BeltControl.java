// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class BeltControl extends CommandBase {
  /** Creates a new BeltControl. */
  int Mode;
  Boolean on, ready, end, isIntake;

  // if you are using belt with intake, pass in isIntake as true
  public BeltControl(Boolean on, boolean isIntake) {
    end = false;
    this.on = on; 
    this.isIntake = isIntake;
    ready = true;
    addRequirements(RobotContainer.m_belt);
  }

  public BeltControl(int Mode) {
    end = false;
    this.Mode = Mode;
    ready = false;
    addRequirements(RobotContainer.m_belt);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (ready) {
      RobotContainer.m_belt.set(on, isIntake);
    } else {
      RobotContainer.m_belt.set(Mode);
    }
    end = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return end;
  }
}
