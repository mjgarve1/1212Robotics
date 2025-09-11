// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
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

  public static class ModuleConstants {
    public static double kWheelDiameterMeters = Units.inchesToMeters(4);

    public static final double kDriveMotorGearRatio = 1/6.75;
    public static final double kTurningMotorGearRatio = (1/(150.0/7));
    
    
    public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
    
    
    //Used as position conversion factor
    public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
    
    public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter/60;
    public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad/60;


    //wtf this do
    //its our p term for the pid for turning - J
    public static final double kPTurning = 0.3;
    
  }
  public static class LadderConstants {
    public static final int kLiftMotorPort = 50;


    //We need to test different values
    public static double kLiftPVal = 1.0; // 3
    public static double kLiftIVal = 0.02; // 1.5
    public static double kLiftDVal = 0.0; //0.2

    //We need to find the points on the ladder for the encoder - J
    /// L1 .46m
    /// L2 .81m
    /// L3 1.21m
    /// L4 1.83m
    /// height at encoder 0 - 
    /// height at max -13.5 - 1.27
    
    // converts encoder value to rotations
    public static double kRotationsPerMeter = -13.5/1.27;

    //height of chassy in meters
    public static double kHeightOfChassy = 0.1905;

    //total offset combining chassy height and gap between ladder and top of l4
    public static double kL4Offset = 0.5461;
    public static double kMidOffset = 0.3429;

    //heights of reef levels in meters
    public static double kL4Height = 1.83;
    public static double kL3Height = 1.21;
    public static double kL2Height = .81;
    public static double kL1Height = .46;

    //setPoints subtracting an offset from the height and converting into rotations
    public static double kLiftHighSetPoint = 14.2; //Y
    public static double kLiftMidSetPoint = 7.954; //B
    public static double kLiftLowSetPoint = 0; //A
    public static double kLiftTroughSetPoint = 2.691; //X

    //recieve is assumed to be 0
    public static double kLiftRecieveSetPoint = 0;
//limits
    public static double kLadderBottom = 0;
    public static double kLadderTop = 14.2;

    //more speed going up 
    public static double kLiftSpeedUp = 0.5;
    public static double kliftSpeedDown = 0.5;
    public static double kStop = 0;

    //range between encoder and setpoint on when to stop for auto
    public static double kSetPointProximity = 0.25;

  }

  public static class IntakeConstants {
    public static int kIntakeMotorPort = 2;

    public static int proxSensorPort = 0;

    public static double kIntakeSpeed = 1;
  }
  
  public static class ClimbConstants {
    public static int kClimbMotorPort = 52;

    public static double kClimbOutSpeed = 0.5;
    public static double kClimbInSpeed = -0.5;

    public static double kClimbInStop = 0;
    public static double kOutStop = 0;

    public static double kClimbPVal = 5;
    public static double kClimbIVal = 5;
    public static double kclimbDVal = 0.2;
    
  }
  // public static class LimelightConstants{
  //   /*
  //   /Height in meters
  //   /when using april tags for distance, all units should be in meters - J
  //   / Coral Station ID: 1, 2, 12, 13 1.35 meters
  //   / Processor ID: 3, 16 1.17 m
  //   / Reef ID: 6 - 11, 17-22 .17 m
  //   / Barge ID: 4, 5, 14, 15 1.78 m
  //   */
  //   private static double[] kAprilTagHeight = {
  //     0, //indexing starts at 0, so we just skip that in the array. - J
  //     1.35, //1 
  //     1.35, //2
  //     1.17, //3
  //     1.78, //4
  //     1.78, //5
  //     .17, //6
  //     .17, //7
  //     .17, //8
  //     .17, //9
  //     .17, //10
  //     .17, //11
  //     1.35, //12
  //     1.35, //13
  //     1.78, //14
  //     1.78, //15
  //     1.17, //16
  //     .17, //17
  //     .17, //18
  //     .17, //19
  //     .17, //20
  //     .17, //21
  //     .17, //22
  //   };
  // }

  public static class DriveConstants{
    // Distance between right and left wheels
    public static final double kTrackWidth = Units.inchesToMeters(15.5);
    // Distance between front and back wheels
    public static final double kWheelBase = Units.inchesToMeters(26.0);
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
      new Translation2d(-kTrackWidth / 2, kWheelBase / 2),
      new Translation2d(kTrackWidth/ 2, kWheelBase / 2),
      new Translation2d(-kTrackWidth / 2, -kWheelBase / 2),
      new Translation2d(kTrackWidth / 2, -kWheelBase / 2)
      );



    public static final double kPhysicalMaxSpeedMetersPerSecond = 3;
    public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

    public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond;
    public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = 
            kPhysicalMaxAngularSpeedRadiansPerSecond / 2;
    public static final double kTeleDriveMaxAccelerationUnitsPerSecond = kPhysicalMaxSpeedMetersPerSecond;
    public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 5;

    public static final double kFineTurning = 3;
    public static final double kFineDriving = 8;
    
    
    public static final int kFrontLeftDriveMotorPort = 11;
    public static final int kBackLeftDriveMotorPort = 3; 
    public static final int kFrontRightDriveMotorPort = 4;
    public static final int kBackRightDriveMotorPort = 10;

    public static final int kFrontLeftTurningMotorPort = 13;
    public static final int kBackLeftTurningMotorPort = 5; 
    public static final int kFrontRightTurningMotorPort = 14;
    public static final int kBackRightTurningMotorPort = 8;

    //Try messing with these reversed/not reversed values some more
    //look at what the shuffleboard values are vs what you want them to be
    public static final boolean kFrontLeftTurningEncoderReversed = false;
    public static final boolean kBackLeftTurningEncoderReversed = false; 
    public static final boolean kFrontRightTurningEncoderReversed = false;
    public static final boolean kBackRightTurningEncoderReversed = false;

    public static final boolean kFrontLeftDriveEncoderReversed = false;
    public static final boolean kBackLeftDriveEncoderReversed = false;
    public static final boolean kFrontRightDriveEncoderReversed = false;
    public static final boolean kBackRightDriveEncoderReversed = false;

    public static final int kFrontLeftDriveAbsoluteEncoderPort = 21;
    public static final int kBackLeftDriveAbsoluteEncoderPort = 20;
    public static final int kFrontRightDriveAbsoluteEncoderPort = 23; 
    public static final int kBackRightDriveAbsoluteEncoderPort = 22;

    public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = true;
    public static final boolean kBackLeftDriveAbsoluteEncoderReversed = true;
    public static final boolean kFrontRightDriveAbsoluteEncoderReversed = true;
    public static final boolean kBackRightDriveAbsoluteEncoderReversed = true;
    //ZERO CANCODERS USING PHOENIX TUNER X INSTEAD
    public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = 0; //21
    public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = 0; //20
    public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = 0; //23
    public static final double kBackRightDriveAbsoluteEncoderOffsetRad = 0;  //22
  }

  public static final class OIConstants {
    public static final int kDriverControllerOnePort = 0;
    public static final int kDriverControllerTwoPort = 1;

    //CHECK CONTROLLER VALUES
    public static final int kDriverYAxis = 1;
    public static final int kDriverXAxis = 0;
    public static final int kDriverRotAxisXbox = 4;
    public static final int kDriverRotAxisJoystick = 2;
    public static final int kSpinIntakeInAxis = 2;
    public static final int kSpinIntakeOutAxis = 3;


    ///////////////////////buttons\\\\\\\\\\\\\\\\\
    //////////////////////////Controller One\\\\\\\\\\\\\\\\\\\\\

    //A
    public static final int kResetGyroButton =1;
    //Y
    public static final int kFineTurningButton = 4;

    //X
    public static final int kDriverFieldOrientedButtonIdx = 3;

    //LB
    public static final int kClimberOut = 5;

    //RB
    public static final int kClimberIn = 6;

    
    /////////////////////////////////Controller Two\\\\\\\\\\\\\\
    //A
    public static final int kLiftLowButton = 1;
    
    //B
    public static final int kLiftMidButton = 2;
    
    //Y
    public static final int kLiftHighButton = 4;

    //X
    public static final int kliftTroughButton = 3;

    //RB
    public static final int kIntakeInButton = 6;

    //LB
    public static final int kIntakeOutButton = 5;

    //three lines
    public static final int kLiftResetEncoderButton = 8;

    //d-pad up
    public static final int kIntakeInPad = 0;

    //d-pad down
    public static final int kIntakeOutPad = 180;

    

    
    
    public static final double kDeadband = 0.15;
}
  
public static final class AutoConstants{
  //isn't used yet but it could be - J
  public static boolean isCompetition = false;

  public static double kAutoTranslationP = 5.0;
  public static double kAutoRotationP = 2.0;

  // private distance to calculate speed.
  public static double kMidDriveForwardDistance = Units.inchesToMeters(75);
  public static double kMidDriveForwardTime = 5.0;
  public static double kMidDriveForwardSpeed = kMidDriveForwardDistance / kMidDriveForwardTime;

  public static double kLeftDriveForwardDistance = Units.inchesToMeters(61.0);
  public static double kLeftDriveForwardTime = 5.0;
  public static double kLeftDriveForwardSpeed = kLeftDriveForwardDistance / kLeftDriveForwardTime;
}
}
//https://software-metadata.revrobotics.com/REVLib-2025.json