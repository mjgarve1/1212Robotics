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
    // Distance between right and left wheels, the Y axis
    public static final double kTrackWidth = Units.inchesToMeters(15.5);
    // Distance between front and back wheels, the X axis
    public static final double kWheelBase = Units.inchesToMeters(26.0);
    
    // X, Y positions of the wheels relative to the center of the robot
    // The order this is defined is the order that states are returned to us
    // Order goes: Front Left, Front Right, Back Left, Back Right
    // Positive X means going to the front of the bot
    // Positive Y means going to the left of the bot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
      
      new Translation2d( kWheelBase / 2,  kTrackWidth / 2), // Front Left Wheel, Negative X, Positive Y
      new Translation2d( kWheelBase / 2, -kTrackWidth / 2), // Front Right Wheel, Positive X, Positive Y
      new Translation2d(-kWheelBase / 2,  kTrackWidth / 2), // Back Left Wheel, Negative X, Negative Y
      new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)  // Back Right Wheel, Positive X, Negative Y
      );

    // The actual maximum possible speed for the robot
    public static final double kPhysicalMaxSpeedMetersPerSecond = 3;

    // The actual maximum possible rotation speed of the robot
    public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * Math.PI;

    // The maximum speed the robot should go
    public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond;

    // The maximum rotational speed of the wheels the robot should go
    public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond / 2;

    // The maximum acceleration or deceleration the robot should have (how long does it take the robot to go from 0 to max speed)
    public static final double kTeleDriveMaxAccelerationUnitsPerSecond = kPhysicalMaxSpeedMetersPerSecond;

    // The maximum acceleartion or deceleration the orientation of the wheels should have (how long does it take for the wheel to reach maximum rotational speed)
    public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 5;

    public static final double kDriveP = 0.04;
    public static final double kDriveI = 0.0;
    public static final double kDriveD = 0.0;

    public static final double kTurningP = 1.0;
    public static final double kTurningI = 0.0;
    public static final double kTurningD = 0.0;

    // How much should fine driving slow down the speed of the robot?
    public static final double kFineDriving = 8;

    // How much should fine driving slow down the rotation of the robot?
    public static final double kFineTurning = 3;
    
    // Swerve module information:
    // Port for movement motor
    // Reversed movement encoder?
    // Port for rotation motor
    // Reversed rotation encoder?
    // Port for absolute encoder
    // Absolute encoder reversed?
    // Absolute encoder offset (should be 0 thanks to Phoenix Tuner X)

    public static final int     kFrontLeftDriveMotorPort                  = 11;
    public static final boolean kFrontLeftDriveEncoderReversed            = false;
    public static final int     kFrontLeftTurningMotorPort                = 13;
    public static final boolean kFrontLeftTurningEncoderReversed          = false;
    public static final int     kFrontLeftDriveAbsoluteEncoderPort        = 21;
    public static final boolean kFrontLeftDriveAbsoluteEncoderReversed    = true;
    public static final double  kFrontLeftDriveAbsoluteEncoderOffsetRad   = 0;


    public static final int     kBackLeftDriveMotorPort                   = 3;
    public static final boolean kBackLeftDriveEncoderReversed             = false;
    public static final int     kBackLeftTurningMotorPort                 = 5;
    public static final boolean kBackLeftTurningEncoderReversed           = false;
    public static final int     kBackLeftDriveAbsoluteEncoderPort         = 20;
    public static final boolean kBackLeftDriveAbsoluteEncoderReversed     = true;
    public static final double  kBackLeftDriveAbsoluteEncoderOffsetRad    = 0;

    public static final int     kFrontRightDriveMotorPort                 = 4;
    public static final boolean kFrontRightDriveEncoderReversed           = false;
    public static final int     kFrontRightTurningMotorPort               = 14;
    public static final boolean kFrontRightTurningEncoderReversed         = false;
    public static final int     kFrontRightDriveAbsoluteEncoderPort       = 23; 
    public static final boolean kFrontRightDriveAbsoluteEncoderReversed   = true;
    public static final double  kFrontRightDriveAbsoluteEncoderOffsetRad  = 0;

    public static final int     kBackRightDriveMotorPort                  = 10;
    public static final boolean kBackRightDriveEncoderReversed            = false;
    public static final int     kBackRightTurningMotorPort                = 8;
    public static final boolean kBackRightTurningEncoderReversed          = false;
    public static final int     kBackRightDriveAbsoluteEncoderPort        = 22;
    public static final boolean kBackRightDriveAbsoluteEncoderReversed    = true;
    public static final double  kBackRightDriveAbsoluteEncoderOffsetRad   = 0;
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

    //Controller One Axis Definitions
    public static final int kRobotForwardAxis   = XboxControllerMappingConstants.LEFT_STICK_Y;
    public static final int kRobotSidewaysAxis  = XboxControllerMappingConstants.LEFT_STICK_X;
    public static final int kRobotRotateAxis    = XboxControllerMappingConstants.RIGHT_STICK_X;

    //Controller One Button Definitions
    public static final int kResetGyroButton              = XboxControllerMappingConstants.A;
    public static final int kFineTurningButton            = XboxControllerMappingConstants.Y;
    public static final int kDriverFieldOrientedButtonIdx = XboxControllerMappingConstants.X;
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