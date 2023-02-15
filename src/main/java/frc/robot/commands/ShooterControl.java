
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class ShooterControl extends CommandBase {
  /** Creates a new ShooterControl. */
  int Mode;
  Boolean on;
  Boolean Ready; 
  Boolean End;
  public ShooterControl(boolean on) {
    // Use addRequirements() here to declare subsystem dependencies.
    End = false;
    this.on = on;
    Ready = true;
    addRequirements(RobotContainer.m_shooter);
  }

  public ShooterControl(int Mode) {
    End = false;
    this.Mode = Mode;
    Ready = false;
    addRequirements(RobotContainer.m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(Ready){
      RobotContainer.m_shooter.set(on);
    } else {
      RobotContainer.m_shooter.set(Mode);
    }
    End = true;
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
    return End;
  }
}
