// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static class ModuleConstants {
    public static double kWheelDiameterMeters = Units.inchesToMeters(4);

    public static final double kDriveMotorGearRatio = 1 / 6.75;
    public static final double kTurningMotorGearRatio = (1 / (150.0 / 7));
    public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
    public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
    public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
    public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
    public static final double kPTurning = 0.3;

  }

  public static class HerderConstants {
    public static int kHerderMotorPort = 50;
    public static int kWinchMotorPort = 52;

    public static double kHerderOutSpeed = 0.9; // changed from 0.1
    public static double kHerderInSpeed = -1; // changed from -0.1

  }
 
  public static class ShooterConstants {
    public static int kShooter1MotorPort = 30;
    public static int kShooter2MotorPort = 31;

    public static double kShooterMotorSpeed = -0.7; //changed direction to neg
    public static double kShooterMotorSpeedAuto = -0.7; //changed direction to neg, changed from .85
    public static double kBackShooterMotorSpeed = 0.2; //changed from -0.5 to 0.5
    public static final Pose2d RED_GOAL_POSE = new Pose2d(11.915, 4.03, new Rotation2d());
    public static final Pose2d BLUE_GOAL_POSE = new Pose2d(4.603, 4.03, new Rotation2d());

    //TODO: Find max and min goal distance empirically by testing the robot on the field and seeing what the range of distances is that the robot can reliably shoot from
    public static final double kMaxGoalDistance = 8.0;
    public static final double kMinGoalDistance = 3.0;

  }
  public static class BeltConstants {
    public static int kBelt1MotorPort = 40;
    public static int kBelt2MotorPort = 41;

    public static double kBeltInSpeed = 0.8; // changed from .5
    public static double kBeltOutSpeed = -1; // 0.5 to 0.8 to 1

  }

  public static class DriveConstants {
    // Distance between right and left wheels
    public static final double kTrackWidth = Units.inchesToMeters(15.5);
    // Distance between front and back wheels
    public static final double kWheelBase = Units.inchesToMeters(26.0);

  // X, Y positions of the wheels relative to the center of the robot
  // The order this is defined is the order that states are returned to us
  // Order goes: Front Left, Front Right, Back Left, Back Right
  // WPILib convention: X = forward/back, Y = left/right
  public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
    // Front Left  (forward, left)
    new Translation2d(+kWheelBase / 2.0, +kTrackWidth / 2.0),
    // Front Right (forward, right)
    new Translation2d(+kWheelBase / 2.0, -kTrackWidth / 2.0),
    // Back Left   (backward, left)
    new Translation2d(-kWheelBase / 2.0, +kTrackWidth / 2.0),
    // Back Right  (backward, right)
    new Translation2d(-kWheelBase / 2.0, -kTrackWidth / 2.0)
  );

    public static final double kPhysicalMaxSpeedMetersPerSecond = 7;
    public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

    public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond;
    public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond / 2;
    public static final double kTeleDriveMaxAccelerationUnitsPerSecond = kPhysicalMaxSpeedMetersPerSecond;
    public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 5;

    public static final double kFineTurning = 1;
    public static final double kFineDriving = 1;

    public static final double kAimAtGoalP = 4.0;

    public static final int kFrontLeftDriveMotorPort = 11;
    public static final int kBackLeftDriveMotorPort = 3;
    public static final int kFrontRightDriveMotorPort = 4;
    public static final int kBackRightDriveMotorPort = 18;

    public static final int kFrontLeftTurningMotorPort = 13;
    public static final int kBackLeftTurningMotorPort = 5;
    public static final int kFrontRightTurningMotorPort = 14;
    public static final int kBackRightTurningMotorPort = 10;

   

    // Try messing with these reversed/not reversed values some more
    // look at what the shuffleboard values are vs what you want them to be
    public static final boolean kFrontLeftTurningEncoderReversed = true;
    public static final boolean kBackLeftTurningEncoderReversed = true;
    public static final boolean kFrontRightTurningEncoderReversed = true;
    public static final boolean kBackRightTurningEncoderReversed = true;

    public static final boolean kFrontLeftDriveEncoderReversed = true;
    public static final boolean kBackLeftDriveEncoderReversed = true;
    public static final boolean kFrontRightDriveEncoderReversed = true;
    public static final boolean kBackRightDriveEncoderReversed = true;

    public static final int kFrontLeftDriveAbsoluteEncoderPort = 21;
    public static final int kBackLeftDriveAbsoluteEncoderPort = 20;
    public static final int kFrontRightDriveAbsoluteEncoderPort = 23;
    public static final int kBackRightDriveAbsoluteEncoderPort = 22;

    public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
    public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
    public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
    public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;

    // ZERO CANCODERS USING PHOENIX TUNER X INSTEAD
    public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = 0; // 21
    public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = 0; // 20
    public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = 0; // 23
    public static final double kBackRightDriveAbsoluteEncoderOffsetRad = 0; // 22
  }

  /************************************************
   * Xbox Controller Mapping Constants
   * 
   * Constants that go here:
   * Xbox controller buttons/axis values
   ************************************************/
  public static final class XboxControllerMappingConstants {
    public static final int LEFT_STICK_X = 0;
    public static final int LEFT_STICK_Y = 1;
    public static final int RIGHT_STICK_X = 4;
    public static final int RIGHT_STICK_Y = 5;
    public static final int A = 1;
    public static final int B = 2;
    public static final int X = 3;
    public static final int Y = 4;
    public static final int LEFT_BUMPER = 5;
    public static final int RIGHT_BUMPER = 6;
    public static final int SELECT = 7;
    public static final int START = 8;
    public static final int LEFT_STICK_BUTTON = 9;
    public static final int RIGHT_STICK_BUTTON = 10;

    // This is tricky, Xbox controllers have variable triggers
    // that are represented as an "axis", so in this particular
    // case the triggers are mapped to moving the motor
    // that controls the coral intake forwards or backwards.
    // These triggers only ever go from 0->1 unlike the other
    // joystick axis which go from -1->1
    public static final int LEFT_TRIGGER = 2;
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
    public static final int DPAD_UP = 0;
    public static final int DPAD_RIGHT = 90;
    public static final int DPAD_DOWN = 180;
    public static final int DPAD_LEFT = 270;
  }

  /************************************************
   * Operator Interface (OI) Constants
   * 
   * Constants that go here:
   * Controller Numbers
   * Detailed Button/Joystick Mappings
   ************************************************/
  public static final class OIConstants {
    // Not sure this is a good place for this...
    public static final double kControllerAxisDeadband = 0.15;

    // Controller Port Definitions
    public static final int kDriverControllerOnePort = 0;
    public static final int kDriverControllerTwoPort = 1;

    // Controller One Axis Definitions
    public static final int kRobotForwardAxis = XboxControllerMappingConstants.LEFT_STICK_Y;
    public static final int kRobotSidewaysAxis = XboxControllerMappingConstants.LEFT_STICK_X;
    public static final int kRobotRotateAxis = XboxControllerMappingConstants.RIGHT_STICK_X;

    // Controller One Button Definitions
    public static final int kResetGyroButton = XboxControllerMappingConstants.A;
    public static final int kFineTurningButton = XboxControllerMappingConstants.Y;
    public static final int kDriverFieldOrientedButtonIdx = XboxControllerMappingConstants.X;
    public static final int kAimAtGoalButton = XboxControllerMappingConstants.B;


    // Controller Two Axis Definitions
    public static final int kShootFuelButton = XboxControllerMappingConstants.RIGHT_TRIGGER;
    public static final int kHerdFuelButton = XboxControllerMappingConstants.LEFT_TRIGGER;
    public static final int kDumpFuelButton = XboxControllerMappingConstants.LEFT_BUMPER;

    public static final int kWinchAxis = XboxControllerMappingConstants.RIGHT_STICK_Y;
    // winch in button on arms controller
    //public static final int kWinchInButton = XboxControllerMappingConstants.A;

    // Controller Two Button Definitions
    public static final double kTriggerDeadband = 0.25;

  }

  public static final class AutoConstants {
    // isn't used yet but it could be - J
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
// https://software-metadata.revrobotics.com/REVLib-2025.json