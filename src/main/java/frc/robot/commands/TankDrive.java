// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.RobotContainer;

public class TankDrive extends CommandBase {
  /** Creates a new TankDrive. */
  Subsystem s_subsystem;
  double PO;
  public TankDrive(Double PO) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.PO = PO;
    addRequirements(RobotContainer.m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Double RJY = RobotContainer.getDeadZone(1) * PO;
    Double LJY = RobotContainer.getDeadZone(5) * PO;

    RobotContainer.m_drivetrain.tankDrive(LJY, RJY);
  
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}