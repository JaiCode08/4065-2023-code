// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  public CANSparkMax ShooterML = new CANSparkMax(Constants.ShooterConstants.ShooterMotorLCAN, MotorType.kBrushless);
  public CANSparkMax ShooterMR = new CANSparkMax(Constants.ShooterConstants.ShooterMotorRCAN, MotorType.kBrushless);
  int mode;
  Boolean on;

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
    // This method will be called once per scheduler run
    if (on) {
      if(mode == 0){
        ShooterML.set(Constants.ShooterConstants.Speed1);
        
      } else if(mode == 1) {
        ShooterML.set(Constants.ShooterConstants.Speed2);
        
      } else if(mode == 2) {
        ShooterML.set(Constants.ShooterConstants.Speed3);

      } else if(mode == 3) {
        ShooterML.set(Constants.ShooterConstants.SpeedNeg);

      }
    } else {
      ShooterML.set(0);
    }
    SmartDashboard.putBoolean("Shooter On", on);
    SmartDashboard.putNumber("Shooter Mode", mode);
    SmartDashboard.putNumber("Shooter Moter Left", ShooterML.get());
    SmartDashboard.putNumber("Shooter Moter Right", ShooterMR.get());
  }
}
