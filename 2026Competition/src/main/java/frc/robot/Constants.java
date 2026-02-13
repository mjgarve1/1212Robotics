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
    
    // X, Y positions of the wheels relative to the center of the robot
    // The order this is defined is the order that states are returned to us
    // Order goes: Front Left, Front Right, Back Left, Back Right
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
      // Front Left Wheel, Negative X, Positive Y
      new Translation2d(-kTrackWidth / 2, kWheelBase / 2),

      // Front Right Wheel, Positive X, Positive Y
      new Translation2d(kTrackWidth/ 2, kWheelBase / 2),

      //Back Left Wheel, Negative X, Negative Y
      new Translation2d(-kTrackWidth / 2, -kWheelBase / 2),

      // Back Right Wheel, Positive X, Negative Y
      new Translation2d(kTrackWidth / 2, -kWheelBase / 2)
      );



    public static final double kPhysicalMaxSpeedMetersPerSecond = 3;
    public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

    public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond;
    public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond / 2;
    public static final double kTeleDriveMaxAccelerationUnitsPerSecond = kPhysicalMaxSpeedMetersPerSecond;
    public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 5;

    public static final double kFineTurning = 3;
    public static final double kFineDriving = 8;

    public static final double kAimAtGoalP = 4.0; 
    
    
    public static final int kFrontLeftDriveMotorPort = 11;
    public static final int kBackLeftDriveMotorPort = 3; 
    public static final int kFrontRightDriveMotorPort = 4;
    public static final int kBackRightDriveMotorPort = 18;

    public static final int kFrontLeftTurningMotorPort = 13;
    public static final int kBackLeftTurningMotorPort = 5; 
    public static final int kFrontRightTurningMotorPort = 14;
    public static final int kBackRightTurningMotorPort = 10;

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

  /************************************************
  Xbox Controller Mapping Constants
  
  Constants that go here:
  Xbox controller buttons/axis values
  ************************************************/
  public static final class XboxControllerMappingConstants {
    public static final int LEFT_STICK_X        = 0;
    public static final int LEFT_STICK_Y        = 1;
    public static final int RIGHT_STICK_X       = 4;
    public static final int RIGHT_STICK_Y       = 5;
    public static final int A                   = 1;
    public static final int B                   = 2;
    public static final int X                   = 3;
    public static final int Y                   = 4;
    public static final int LEFT_BUMPER         = 5;
    public static final int RIGHT_BUMPER        = 6;
    public static final int SELECT              = 7;
    public static final int START               = 8;
    public static final int LEFT_STICK_BUTTON   = 9;
    public static final int RIGHT_STICK_BUTTON  = 10;

    // This is tricky, Xbox controllers have variable triggers
    // that are represented as an "axis", so in this particular
    // case the triggers are mapped to moving the motor
    // that controls the coral intake forwards or backwards.
    // These triggers only ever go from 0->1 unlike the other
    // joystick axis which go from -1->1
    public static final int LEFT_TRIGGER  = 2;
    public static final int RIGHT_TRIGGER = 3;

    // Another tricky Xbox controller feature, the DPAD
    // is not a set of 4 buttons, it is essentially
    // a potentiometer that goes from 0->360 clockwise
    // So when 0 is read that means up is pressed
    // when 90 is read that means right is pressed
    // when 180 is read that means down is pressed
    // when 270 is read that means left is pressed
    // This can also be read directly and you can get
    // up-right being pressed if it reads 45 etc...
    public static final int DPAD_UP     = 0;
    public static final int DPAD_RIGHT  = 90;
    public static final int DPAD_DOWN   = 180;
    public static final int DPAD_LEFT   = 270;
  }

  /************************************************
  Operator Interface (OI) Constants
  
  Constants that go here:
  Controller Numbers
  Detailed Button/Joystick Mappings
  ************************************************/
  public static final class OIConstants
  {
    //Not sure this is a good place for this...
    public static final double kControllerAxisDeadband = 0.15;

    //Controller Port Definitions
    public static final int kDriverControllerOnePort = 0;
    public static final int kDriverControllerTwoPort = 1;

    // april tag follower button
    public static final int kAimAtGoalButton = XboxControllerMappingConstants.RIGHT_STICK_BUTTON;

    //Controller One Axis Definitions
    public static final int kRobotForwardAxis   = XboxControllerMappingConstants.LEFT_STICK_Y;
    public static final int kRobotSidewaysAxis  = XboxControllerMappingConstants.LEFT_STICK_X;
    public static final int kRobotRotateAxis    = XboxControllerMappingConstants.RIGHT_STICK_X;

    //Controller One Button Definitions
    public static final int kResetGyroButton              = XboxControllerMappingConstants.A;
    public static final int kFineTurningButton            = XboxControllerMappingConstants.Y;
    public static final int kDriverFieldOrientedButtonIdx = XboxControllerMappingConstants.X;
    public static final int kClimberOut                   = XboxControllerMappingConstants.LEFT_BUMPER;
    public static final int kClimberIn                    = XboxControllerMappingConstants.RIGHT_BUMPER;
    
    //Controller Two Axis Definitions
    public static final int kLadderAxis         = XboxControllerMappingConstants.LEFT_STICK_Y;
    public static final int kSpinIntakeInAxis   = XboxControllerMappingConstants.LEFT_TRIGGER;
    public static final int kSpinIntakeOutAxis  = XboxControllerMappingConstants.RIGHT_TRIGGER;

    //Controller Two Button Definitions
    public static final int kLiftLowButton          = XboxControllerMappingConstants.A;
    public static final int kLiftMidButton          = XboxControllerMappingConstants.B;
    public static final int kLiftHighButton         = XboxControllerMappingConstants.Y;
    public static final int kliftTroughButton       = XboxControllerMappingConstants.X;
    public static final int kIntakeInButton         = XboxControllerMappingConstants.RIGHT_BUMPER;
    public static final int kIntakeOutButton        = XboxControllerMappingConstants.LEFT_BUMPER;
    public static final int kLiftResetEncoderButton = XboxControllerMappingConstants.START;
    public static final int kUnlockLadderButton     = XboxControllerMappingConstants.SELECT;
    public static final int kIntakeInPad            = XboxControllerMappingConstants.DPAD_UP;
    public static final int kIntakeOutPad           = XboxControllerMappingConstants.DPAD_DOWN;
  }
  
  public static final class AutoConstants
  {
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