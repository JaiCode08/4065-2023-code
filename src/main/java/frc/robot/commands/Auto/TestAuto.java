// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.Time;

/** Add your docs here. */
public class TestAuto {
    public Command cmd() {
        SequentialCommandGroup cmd = new SequentialCommandGroup(RobotContainer.m_drivetrain.loadPathRam("deploy/pathplanner/generatedJSON/test.wpilib.json", true));
        return cmd;
    }
}