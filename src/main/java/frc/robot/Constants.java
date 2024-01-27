// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// thde WPILib BSD license file in the root directory of this project.


package frc.robot; 

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
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
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDeadband = 0;
    public static final int kDriverFieldOrientedButtonIdx = 1;
    public static int kDriverYAxis;
    public static int kDriverXAxis;
    public static int kDriverRotAxis;
  }
  public static final class DriveTrainConstants {
    public static final int kLeftFrontEncoderCANID = 11;
    public static final int kLeftFrontDriveMotorCANID = 12;
    public static final int kLeftFrontStearingMotorCANID = 13;
    public static final boolean kFrontLeftDriveMotorReversed = false;
    public static final boolean kFrontLeftSteeringMotorReversed = false;
    public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = -0.428;
    public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
    
    public static final int kRightFrontEncoderCANID = 21;
    public static final int kRightFrontDriveMotorCANID = 22;
    public static final int kRightFrontStearingMotorCANID = 23;
    public static final boolean kFrontRightDriveMotorReversed = false;
    public static final boolean kFrontRightSteeringMotorReversed = false;
    public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = 0.025;
    public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;

    public static final int kLeftRearEncoderCANID = 31;
    public static final int kLeftRearDriveMotorCANID = 32;
    public static final int kLeftRearStearingMotorCANID = 33;
    public static final boolean kLeftRearDriveMotorReversed = false;
    public static final boolean kLeftRearSteeringMotorReversed = false;
    public static final double kLeftRearDriveAbsoluteEncoderOffsetRad = 0.243;
    public static final boolean kLeftRearDriveAbsoluteEncoderReversed = false;
    
    public static final int kRightRearEncoderCANID = 41;
    public static final int kRightRearDriveMotorCANID = 42;
    public static final int kRightRearStearingMotorCANID = 43;

    public static final boolean kRightRearDriveMotorReversed = false;
    public static final boolean kRightRearSteeringMotorReversed = false;
    public static final double kRightRearDriveAbsoluteEncoderOffsetRad = 0.261;
    public static final boolean kRightRearDriveAbsoluteEncoderReversed = false;

    //limit Current to avoid drawing batter too low & getting chatter
    public static final int SparkMaxCurrentLimit = 30;

  
    //sysID values
    public static final double ksVolts = 0.2236;
    public static final double kvVoltSecondsPerMeter = 1.5654;
    public static final double kaVoltSecondsSquaredPerMeter = 0.22128;
    public static final double kpDriveVel = 0.023927;

    // differential drive kinmatics
    public static final double kTrackWidthMeters = Units.inchesToMeters(23);
    public static final double kWheelBase = Units.inchesToMeters(23.25);
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
      new Translation2d(kWheelBase / 2, -kTrackWidthMeters / 2),
      new Translation2d(kWheelBase / 2, kTrackWidthMeters / 2),
      new Translation2d(-kWheelBase / 2, -kTrackWidthMeters / 2),
      new Translation2d(-kWheelBase / 2, kTrackWidthMeters / 2));
 

  

    //public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackWidthMeters);

    //max velocity and accleration
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;

    //Reasonable baseline values for a RAMSETE follower in units of meters and seconds
    public static final double kRamsetB = 2;
    public static final double kRamseteZeta = 0.7;

    //Gear Ratio
    public static final double kGearRatio = 8.5;
    public static final double kWheelRadiusInches = 3;
    
    // converting ticks to meters
    public static final double kLinearDistanceConversionFactor = (Units.inchesToMeters(1/(kGearRatio*2*Math.PI*Units.inchesToMeters(kWheelRadiusInches))*10));
    public static final double kPhysicalMaxSpeedMetersPerSecond = 5;
    public static final double kTeleDriveAccelerationUnitsPerSecond = 0;
    public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 1;
    public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 1;
  }

  public static final class SwerveModuleConstants {
      public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
      public static final double kDriveMotorGearRatio = 1 / 6.75;
      public static final double kTurningMotorGearRatio = 1 / 12.;
      public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
      public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
      public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
      public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
      public static final double kPTurning = 0.5;
      
  }
}
