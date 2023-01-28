// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
   public static final int pneumaticHub = 4;
   public static final int s_solenoid = 0;
   public static final int leftMotor = 2;
   public static final int rightMotor = 1;
   public static final int swingArm = 3;
   public static final int clawMotor = 0;
   public static final int potPort = 3;
   public static final int usbCamera = 1;

   public static final class DriveConstants {
      public static final int RightMaster = 2;
      public static final int RightSlave = 4;
      public static final int LeftMaster = 1;
      public static final int LeftSlave = 3;
      public static final double kGearRatio = 5.45;
      public static final double kWheelRadiusInches = 2;
      public static final double kWheelCircumferenceInches = 2 * Math.PI * kWheelRadiusInches;
      public static final double kTrackWidthMeters = Units.inchesToMeters(26);
      public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackWidthMeters);
   }

   public static final class AutoConstants {
      public static final double ksVolts = 0.1925;
      public static final double kvVoltSecondsPerMeter = 1.7869;
      public static final double kaVoltSecondsSquaredPerMeter = 0.10251;
      public static final double kPDriveVel = 1.5 ; // 0.040403
            
      public static final double kMaxSpeedMetersPerSecond = 3;
      public static final double kMaxAccelMetersPerSecondSqaured = 3;

      public static final double kRamseteB = 2;
      public static final double kRamseteZeta = 0.7;

      public static final double kEncoderFullRev = 2048;
      

   }
}