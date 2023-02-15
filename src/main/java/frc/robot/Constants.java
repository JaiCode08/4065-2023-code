// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
   public static final class DriveConstants {
      public static final int RightMaster = 9;
      public static final int RightSlave = 10;
      public static final int LeftMaster = 18;
      public static final int LeftSlave = 19;
      public static final double kGearRatio = 10.9;
      public static final double kWheelRadiusInches = 3;
      public static final double kWheelCircumferenceInches = 2 * Math.PI * kWheelRadiusInches;
      public static final double kTrackWidthMeters = Units.inchesToMeters(26);
      public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackWidthMeters);
   }

   public static final class ShooterConstants {
      public static final int ShooterMotorRCAN = 17;
      public static final int ShooterMotorLCAN = 12;
      public static final double Speed1 = 0.2;
      public static final double Speed2 = 0.6;
      public static final double Speed3 = 1;
      public static final double SpeedNeg = -0.2;  
   }

   public static final class BeltConstants {
      public static final int BeltMotorCAN = 13;
      public static final double BeltSpeed = 0.75;
      public static final double NegBeltSped = -0.5;
   }

   public static final class Intake {
      public static final int IntakeMotor = 14;
      public static final int pneumaticHub = 0;
   }

   public static final class AutoConstants {
      public static final double ksVolts = 0.14534;
      public static final double kvVoltSecondsPerMeter = 4.0318;
      public static final double kaVoltSecondsSquaredPerMeter = 0.42035;
      public static final double kPDriveVel = 0.043262; // PID Mode - kp val ---> 0.13009 STILL NEED TO FIX THE PID value
            
      public static final double kMaxSpeedMetersPerSecond = 3;
      public static final double kMaxAccelMetersPerSecondSqaured = 3;

      public static final double kRamseteB = 2;
      public static final double kRamseteZeta = 0.7;

      public static final double kEncoderFullRev = 2048;
      
      public static final HashMap<String, Command> testEventMap = new HashMap<>();
      public static final HashMap<String, Command> anotherTestEventMap = new HashMap<>();
   }

   public static final class Other {
      public static final int blinkInPWM = 0;
      public static final I2C.Port colorSensorPort = I2C.Port.kOnboard;
      public static final double detectThreshold = 0.22;
   }
}