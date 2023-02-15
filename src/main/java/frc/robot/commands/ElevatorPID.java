// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class ElevatorPID extends CommandBase {
  /** Creates a new ElevatorPID. */
  PIDController pid = new PIDController(0.01, 0, 0);
  AnalogInput input = new AnalogInput(0);
  AnalogPotentiometer potVal = new AnalogPotentiometer(input);

  boolean end;
  double pidSetPoint;

  public ElevatorPID(double pidSetPoint) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_elevator);
    this.pidSetPoint = pidSetPoint;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.m_elevator.setMotorSpeed(pid.calculate(potVal.get(), pidSetPoint));
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
