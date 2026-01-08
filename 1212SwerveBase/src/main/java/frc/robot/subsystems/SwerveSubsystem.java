// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pathplanner.lib.config.RobotConfig;
import com.studica.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class SwerveSubsystem extends SubsystemBase {
  /** Creates a new SwerveSubsystem. */

   private final SwerveModule frontLeft = new SwerveModule(
        DriveConstants.kFrontLeftDriveMotorPort, 
        DriveConstants.kFrontLeftTurningMotorPort, 
        DriveConstants.kFrontLeftDriveEncoderReversed, 
        DriveConstants.kFrontLeftTurningEncoderReversed, 
        DriveConstants.kFrontLeftDriveAbsoluteEncoderPort, 
        DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad, 
        DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed
        );
    private final SwerveModule frontRight = new SwerveModule(
        DriveConstants.kFrontRightDriveMotorPort, 
        DriveConstants.kFrontRightTurningMotorPort, 
        DriveConstants.kFrontRightDriveEncoderReversed, 
        DriveConstants.kFrontRightTurningEncoderReversed, 
        DriveConstants.kFrontRightDriveAbsoluteEncoderPort, 
        DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad, 
        DriveConstants.kFrontRightDriveAbsoluteEncoderReversed
        );
    private final SwerveModule backLeft = new SwerveModule(
        DriveConstants.kBackLeftDriveMotorPort, 
        DriveConstants.kBackLeftTurningMotorPort, 
        DriveConstants.kBackLeftDriveEncoderReversed, 
        DriveConstants.kBackLeftTurningEncoderReversed, 
        DriveConstants.kBackLeftDriveAbsoluteEncoderPort, 
        DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad, 
        DriveConstants.kBackLeftDriveAbsoluteEncoderReversed
        );
    private final SwerveModule backRight = new SwerveModule(
        DriveConstants.kBackRightDriveMotorPort, 
        DriveConstants.kBackRightTurningMotorPort, 
        DriveConstants.kBackRightDriveEncoderReversed, 
        DriveConstants.kBackRightTurningEncoderReversed, 
        DriveConstants.kBackRightDriveAbsoluteEncoderPort, 
        DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad, 
        DriveConstants.kBackRightDriveAbsoluteEncoderReversed
        );

    // Gyro to monitor the heading of the robot
    private final AHRS gyro = new AHRS(AHRS.NavXComType.kMXP_SPI, AHRS.NavXUpdateRate.k50Hz);

    //Set up initial Odometry
    private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(DriveConstants.kDriveKinematics, getRotation2d(),//new Rotation2d(0),
        new SwerveModulePosition[]{frontLeft.getPosition(),frontRight.getPosition(),backLeft.getPosition(),backRight.getPosition()});

    private ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0,0,0);

    
  public SwerveSubsystem() {
    }

    public void zeroHeading() {
        gyro.reset();
    }

    //gets the heading returned as the gyro reading remainder after being divided by 360
    //that way it always reads from 0 to 360
    //or 0 to -360
    public double getHeading() {
        //this being negative screws with the gyro. - J
        return Math.IEEEremainder(gyro.getAngle(), 360);
    }

    public double getHeadingRadians(){
      return Units.degreesToRadians(getHeading());
    }
    //returns as rotation2d object (in radians)
    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    //returns Pose with x,y, and theta coordinates of robot
    // now uses poseEstimator because of limeLight compatability.
    public Pose2d getPose(){
        return odometer.getPoseMeters();
    }

    //We moved the use of Chassis Speeds from our Swerve Joystick Command to our Swerve Subsytem
    //This will be seen in setModuleStates and in driveRobotRelative (Which is just used for auto currently)
    public void setChassisSpeed( ChassisSpeeds speed){
        chassisSpeeds = speed;
    }

    //gets the chassis speeds relative the robot - J
    public ChassisSpeeds getRobotRelativeSpeeds(){
        return ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds, getRotation2d());
        //return ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds, Rotation2d.fromDegrees(-getHeading()));
    }
    //reset the odometer current theta, module positions
    public void resetPose(Pose2d newPose){
      odometer.resetPosition(
        getRotation2d(),
        new SwerveModulePosition[] {
          frontLeft.getPosition(),
          frontRight.getPosition(),
          backLeft.getPosition(),
          backRight.getPosition()
        },
        newPose);
        
    }

    public void updateOdometry() {
      odometer.update(
        getRotation2d(),
        new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
        });
    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
     //updates odometer based on new positions which take into account turn encoder and drive encoder along with position on robot
     updateOdometry();
     
     //SmartDashboard.putString("frontLeft", frontLeft.getPosition().toString());
     //SmartDashboard.putString("frontRight", frontRight.getPosition().toString());
     //SmartDashboard.putString("backLeft", backLeft.getPosition().toString());
     //SmartDashboard.putString("backRight", backRight.getPosition().toString());
     //SmartDashboard.putString("Robot Heading", getRotation2d().toString());
     //SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
     //SmartDashboard.putNumber("Robot heading", getHeadingRadians());    
  }

public void driveRobotRelative(ChassisSpeeds robotRelativeSpeed){
    SwerveModuleState[] desiredStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(ChassisSpeeds.fromRobotRelativeSpeeds(robotRelativeSpeed, getRotation2d()));
  SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);

  frontLeft.setDesiredState(desiredStates[0]);
  frontRight.setDesiredState(desiredStates[1]);
  backLeft.setDesiredState(desiredStates[2]);
  backRight.setDesiredState(desiredStates[3]);
}

public void setModuleStates() {

  SwerveModuleState[] desiredStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
  SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);

  frontLeft.setDesiredState(desiredStates[0]);
  frontRight.setDesiredState(desiredStates[1]);
  backLeft.setDesiredState(desiredStates[2]);
  backRight.setDesiredState(desiredStates[3]);

}

}
