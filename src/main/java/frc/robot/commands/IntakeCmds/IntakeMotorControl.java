// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCmds;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class IntakeMotorControl extends CommandBase {
  /** Creates a new IntakeMotorControl. */
  boolean move, isIntake, end;
  public IntakeMotorControl(boolean move, boolean isIntake) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_motorsys);
    this.move = move;
    this.isIntake = isIntake;
    end = false;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.m_motorsys.set(move, isIntake);
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
