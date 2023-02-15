// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Belt extends SubsystemBase {
  /** Creates a new Belt. */
  int mode;
  boolean on, isIntake;

  public Belt(int DefaultSpeedMode) {
    on = false;
    mode = DefaultSpeedMode;
    BeltMotor.setInverted(true);
    BeltMotor.setNeutralMode(NeutralMode.Brake);
  }


  public void set(boolean on, boolean isIntake) {
    this.on = on;
    this.isIntake = isIntake;
  }

  public void set(int Mode) {
    mode = Mode;
  }

  public TalonFX BeltMotor = new TalonFX(Constants.BeltConstants.BeltMotorCAN);
  
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(Constants.Other.colorSensorPort);
  public static Color detectedColor;

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    detectedColor = m_colorSensor.getColor();
    SmartDashboard.putNumber("RED", detectedColor.red);
    SmartDashboard.putNumber("GREEN", detectedColor.green);
    SmartDashboard.putNumber("BLUE", detectedColor.blue);

    RobotContainer.downButton = new POVButton(RobotContainer.XboxC, 180);

    if (RobotContainer.downButton.getAsBoolean() == false) {
      // if belt is being used with intake, do regular action
      if (isIntake == false) {
        if (on) {
          if (mode == 0) {
            BeltMotor.set(ControlMode.PercentOutput, Constants.BeltConstants.BeltSpeed); 
    
          } else if(mode == 1) {
            BeltMotor.set(ControlMode.PercentOutput, Constants.BeltConstants.NegBeltSped);
    
          }
        } else {
          BeltMotor.set(ControlMode.PercentOutput, 0);
        }
      } else {
        // there was a massive difference of the B val between cube and no cube. if the B is higher than 0.25 (detected), stop the belt
        if (detectedColor.blue > Constants.Other.detectThreshold) {
          BeltMotor.set(ControlMode.PercentOutput, 0);
        } else {
          if (on) {
            if (mode == 0) {
              BeltMotor.set(ControlMode.PercentOutput, 0.5); 
      
            } else if(mode == 1) {
              BeltMotor.set(ControlMode.PercentOutput, Constants.BeltConstants.NegBeltSped);
      
            }
          } else {
            BeltMotor.set(ControlMode.PercentOutput, 0);
          }
        }
      }
    } else if (RobotContainer.downButton.getAsBoolean() == true) {
      BeltMotor.set(ControlMode.PercentOutput, -0.5);
    }
    
    SmartDashboard.putBoolean("Belt On", on);
    SmartDashboard.putNumber("Belt Speed", BeltMotor.getMotorOutputPercent());
  }
}
