// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class AutoBalance extends CommandBase {
  /** Creates a new getOnRamp. */
  // double error, currentAngle, drivePower, kP = 0.08; // kp 0.015
  PIDController pid = new PIDController(0.024, 0.001, 0.002);
  double speed;
  double angle;
  public AutoBalance() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("AUTO BALANCING ENABLED!");
    RobotContainer.m_drivetrain.changeRamp(0);
    RobotContainer.m_drivetrain.setBreakMode(); 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("balancing...");
    angle = RobotContainer.m_drivetrain.getPitch();
    speed = pid.calculate(angle, Constants.AutoConstants.balancedGyro);

    if (speed < -0.5) {
      RobotContainer.m_drivetrain.tankDrive(0.429 * 1.1, 0.429 * 1.1);
    } else if (speed > 0.5) {
      RobotContainer.m_drivetrain.tankDrive(-0.429 * 1.1, -0.429 * 1.1);
    } else {
      RobotContainer.m_drivetrain.tankDrive(-speed, -speed);
    }

    
    System.out.println(angle < (Constants.AutoConstants.balancedGyro + 2.5) && (Constants.AutoConstants.balancedGyro - 2.5) < angle);

  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_drivetrain.setBreakMode();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return Math.abs; 
    return angle < (Constants.AutoConstants.balancedGyro + 2.5) && (Constants.AutoConstants.balancedGyro - 2.5) < angle;
  }
}
