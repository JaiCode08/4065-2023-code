// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  public CANSparkMax ShooterML = new CANSparkMax(Constants.ShooterConstants.ShooterMotorLCAN, MotorType.kBrushless);
  public CANSparkMax ShooterMR = new CANSparkMax(Constants.ShooterConstants.ShooterMotorRCAN, MotorType.kBrushless);
  int mode = 0;
  Boolean on = false;

  String shooterMode = "LOW";

  private GenericEntry onValSB = Robot.mainTab.add("SHOOTER", on).withPosition(1, 1).getEntry();
  private GenericEntry shooterModeValSB = Robot.mainTab.add("SHOOTER MODE", shooterMode).withPosition(0, 1).getEntry();

  public Shooter(int DefaultSpeedMode) {
    on = false;
    mode = DefaultSpeedMode;
    ShooterML.setIdleMode(IdleMode.kBrake);
    ShooterMR.setIdleMode(IdleMode.kBrake);
    
    ShooterML.setInverted(true);
    ShooterMR.follow(ShooterML, true);
  }

  public void set(boolean on) {
    this.on = on;
  }

  public void set(int Mode) {
    mode = Mode;
  }

  @Override
  public void periodic() {

    switch(mode) {
      case 0:
        shooterMode = "LOW";
        break;
      case 1:
        shooterMode = "MEDIUM";
        break;
      case 2:
        shooterMode = "HIGH";
        break;
      default:
        System.out.println("ERROR");
    }

    RobotContainer.B1 = new JoystickButton(RobotContainer.XboxC, 11);

    onValSB.setBoolean(on);
    shooterModeValSB.setString(shooterMode);
    // This method will be called once per scheduler run
    if (on) {
      if(mode == 0) {
        ShooterML.setVoltage(Constants.ShooterConstants.Speed1);
        
      } else if(mode == 1) {
        ShooterML.setVoltage(Constants.ShooterConstants.Speed2);
        
      } else if(mode == 2) {
        ShooterML.setVoltage(Constants.ShooterConstants.Speed3);

      }
    } else if (RobotContainer.B1.getAsBoolean() == true) {
      ShooterML.setVoltage(Constants.ShooterConstants.SpeedNeg);
    } else {
      ShooterML.set(0);
    }
  }
}
