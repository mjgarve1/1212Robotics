// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.studica.frc.AHRS;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;

public class SwerveSubsystem extends SubsystemBase {
  /** Creates a new SwerveSubsystem. */

  private final SwerveModule frontLeft = new SwerveModule(
      DriveConstants.kFrontLeftDriveMotorPort,
      DriveConstants.kFrontLeftTurningMotorPort,
      DriveConstants.kFrontLeftDriveEncoderReversed,
      DriveConstants.kFrontLeftTurningEncoderReversed,
      DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
      DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
      DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed);
  private final SwerveModule frontRight = new SwerveModule(
      DriveConstants.kFrontRightDriveMotorPort,
      DriveConstants.kFrontRightTurningMotorPort,
      DriveConstants.kFrontRightDriveEncoderReversed,
      DriveConstants.kFrontRightTurningEncoderReversed,
      DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
      DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
      DriveConstants.kFrontRightDriveAbsoluteEncoderReversed);
  private final SwerveModule backLeft = new SwerveModule(
      DriveConstants.kBackLeftDriveMotorPort,
      DriveConstants.kBackLeftTurningMotorPort,
      DriveConstants.kBackLeftDriveEncoderReversed,
      DriveConstants.kBackLeftTurningEncoderReversed,
      DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
      DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
      DriveConstants.kBackLeftDriveAbsoluteEncoderReversed);
  private final SwerveModule backRight = new SwerveModule(
      DriveConstants.kBackRightDriveMotorPort,
      DriveConstants.kBackRightTurningMotorPort,
      DriveConstants.kBackRightDriveEncoderReversed,
      DriveConstants.kBackRightTurningEncoderReversed,
      DriveConstants.kBackRightDriveAbsoluteEncoderPort,
      DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad,
      DriveConstants.kBackRightDriveAbsoluteEncoderReversed);

  // Gyro to monitor the heading of the robot
  private final AHRS gyro = new AHRS(AHRS.NavXComType.kMXP_SPI, AHRS.NavXUpdateRate.k50Hz);

  // Set up initial Odometry
  private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(DriveConstants.kDriveKinematics,
      new Rotation2d(0),
      new SwerveModulePosition[] { frontLeft.getPosition(), frontRight.getPosition(), backLeft.getPosition(),
          backRight.getPosition() });

  // Since we do not have feedback from the motors as to exactly the speed and
  // heading, we have to estimate it
  // This sets up the estimator
  private final SwerveDrivePoseEstimator m_poseEstimator = new SwerveDrivePoseEstimator(
      DriveConstants.kDriveKinematics,
      new Rotation2d(0),
      new SwerveModulePosition[] {
          frontLeft.getPosition(),
          frontRight.getPosition(),
          backLeft.getPosition(),
          backRight.getPosition()
      }, odometer.getPoseMeters());

  private ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, 0, 0);
  // PID controller for aiming (operates in degrees)
  private PIDController aimPid;
  private RobotConfig config;

  private Integer zeroHeadingBasedOnVision = 0;

  public SwerveSubsystem() {
    // used to link odometry with limelight for better pose estimation.
    LimelightHelpers.SetIMUMode("limelight", 2);

    new Thread(() -> {
      try {
        Thread.sleep(1000);
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
          if (alliance.get() == DriverStation.Alliance.Red) {
            zeroHeading(180);
          } else {
            zeroHeading(0);
          }
          zeroHeadingBasedOnVision = 0;
        }

      } catch (Exception e) {
      }
    }).start();

    // The following is the code to configure the pathplanner auto builder
    try {
      config = RobotConfig.fromGUISettings(); // here it takes the setting we input in the path Planner gui
    } catch (Exception e) {
      e.printStackTrace();
    }

    AutoBuilder.configure(
        this::getPose, // gets a supplier of pose2d
        this::resetPose, // used if the auto needs to reset the pose if reset odometry is checked
        this::getRobotRelativeSpeeds, // uses the chassisSpeeds relative to the robot
        (speeds, feedforwards) -> driveRobotRelative(speeds), // used to command the robot chassis speeds using robot
                                                              // relative speeds
        new PPHolonomicDriveController( // PID controllers for moving and rotating in autonomous.
            new PIDConstants(AutoConstants.kAutoTranslationP, 0.0, 0.0),
            new PIDConstants(AutoConstants.kAutoRotationP, 0.0, 0.0)),
        config, // uses the Robot config to configure the AutoBuilder to the robot specs
        () -> { // I believe there is a chooser that lets us choose which alliance we're on and
                // flips the auot if necessary.
          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this);

  // Aim PID (degrees)
  aimPid = new PIDController(0.5, 0.1, 0.0);
  aimPid.enableContinuousInput(-180.0, 180.0);
  }

  public void zeroHeading(double angleAdjustment) {
    gyro.reset();
    gyro.setAngleAdjustment(-angleAdjustment);
  }

  // gets the heading returned as the gyro reading remainder after being divided
  // by 360
  public double getHeading() {
    SmartDashboard.putNumber("Gyro Angle", -gyro.getAngle());
    return Math.IEEEremainder(-gyro.getAngle(), 360);
  }

  public double getHeadingRadians() {
    return Units.degreesToRadians(getHeading());
  }

  // returns as rotation2d object (in radians)
  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(getHeading());
  }

  // returns Pose with x,y, and theta coordinates of robot
  // now uses poseEstimator because of limeLight compatability.
  public Pose2d getPose() {
    return m_poseEstimator.getEstimatedPosition();
  }

  
  public double getGoalDistance() {
    // distance (meters) between current robot pose and the shooter goal pose
    Translation2d robotTrans = getPose().getTranslation();
    Translation2d goalTrans = ShooterConstants.BLUE_GOAL_POSE.getTranslation();
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && alliance.get() == Alliance.Red) {
      goalTrans = ShooterConstants.RED_GOAL_POSE.getTranslation();
    }
    
    double dx = robotTrans.getX() - goalTrans.getX();
    double dy = robotTrans.getY() - goalTrans.getY();
    return Math.hypot(dx, dy);
  }
  
  // We moved the use of Chassis Speeds from our Swerve Joystick Command to our
  // Swerve Subsytem
  // This will be seen in setModuleStates and in driveRobotRelative (Which is just
  // used for auto currently)
  public void setChassisSpeed(ChassisSpeeds speed) {
    chassisSpeeds = speed;
  }

  // gets the chassis speeds relative the robot - J
  public ChassisSpeeds getRobotRelativeSpeeds() {
    return ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds, getRotation2d());
    // return ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds,
    // Rotation2d.fromDegrees(-getHeading()));
  }

  // reset the odometer current theta, module positions
  public void resetPose(Pose2d newPose) {
    Pose2d pose = newPose;
    m_poseEstimator.resetPosition(getRotation2d(),
        new SwerveModulePosition[] { frontLeft.getPosition(), frontRight.getPosition(), backLeft.getPosition(),
            backRight.getPosition() },
        pose);

  }

  /// this uses the limelight software to update the estimated position of the
  /// robot.
  ///
  public void updateOdometry() {
    m_poseEstimator.update(
        getRotation2d(),
        new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
        });

    boolean doRejectUpdate = false;
    LimelightHelpers.SetRobotOrientation("limelight", m_poseEstimator.getEstimatedPosition().getRotation().getDegrees(),
        0, 0, 0, 0, 0);
    LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
    if (Math.abs(gyro.getRate()) > 720) // if our angular velocity is greater than 720 degrees per second, ignore vision
                                        // updates
    {
      doRejectUpdate = true;
    }
    if (mt1 == null) {
      doRejectUpdate = true;
    } else if (mt1.tagCount == 0) {
      doRejectUpdate = true;
    }
    if (!doRejectUpdate) {
      m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, .7));
      m_poseEstimator.addVisionMeasurement(
          mt1.pose,
          mt1.timestampSeconds);
      if (zeroHeadingBasedOnVision == 10) {
        zeroHeading(mt1.pose.getRotation().getDegrees());
        zeroHeadingBasedOnVision = 11;
      } else if (zeroHeadingBasedOnVision < 10) {
        zeroHeadingBasedOnVision++;
      }
    }
    SmartDashboard.putNumber("Orientation Updated", zeroHeadingBasedOnVision);
    SmartDashboard.putNumber("x pose", m_poseEstimator.getEstimatedPosition().getX());
    SmartDashboard.putNumber("y pose", m_poseEstimator.getEstimatedPosition().getY());
    SmartDashboard.putNumber("rotation pose", m_poseEstimator.getEstimatedPosition().getRotation().getDegrees());
  }

  public Command driveTowardAprilTag() {
    Pose2d targetPose = LimelightHelpers.getTargetPose3d_RobotSpace("limelight").toPose2d();
    PathConstraints constraints = new PathConstraints(DriveConstants.kPhysicalMaxSpeedMetersPerSecond,
        DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond,
        DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond,
        DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);

    return AutoBuilder.pathfindToPose(targetPose, constraints, 0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // updates odometer based on new positions which take into account turn encoder
    // and drive encoder along with position on robot
    updateOdometry();

  }

  public void enableReset() {
    zeroHeadingBasedOnVision = 0;
  }

  public void stopModules() {
    frontLeft.stop();
    frontRight.stop();
    backLeft.stop();
    backRight.stop();
  }

  /**
   * Compute the angular offset (in degrees) between the robot's current heading
   * and the heading required to face the given goal pose. Also publishes
   * desired/offset angles to SmartDashboard for debugging.
   *
   * @param goalPose goal pose in field coordinates
   * @return offset in degrees (robotHeading - desiredHeading)
   */
  public double getAimOffsetDegrees(Pose2d goalPose) {
    Pose2d robotPose = getPose();
    Rotation2d desiredRotation2d = robotPose.relativeTo(goalPose).getTranslation().getAngle()
        .minus(Rotation2d.k180deg);
    double testDegrees = desiredRotation2d.getDegrees();
    double offsetDegrees = robotPose.getRotation().minus(desiredRotation2d).getDegrees();

    SmartDashboard.putNumber("Desired Robot Angle", testDegrees);
    SmartDashboard.putNumber("Offset Angle", offsetDegrees);

    return offsetDegrees;
  }

  /**
   * Compute a turning speed (in the same units the joystick code expects) to aim at the given goal.
   * This uses the internal aim PID controller and also publishes the Goal Speed to SmartDashboard.
   *
   * @param goalPose goal pose in field coordinates
   * @return turning speed (unitless as used by SwerveJoystickCmd)
   */
  public double getAimTurningSpeed(Pose2d goalPose) {
    double offsetDegrees = getAimOffsetDegrees(goalPose);
    double turningSpeed = aimPid.calculate(offsetDegrees, 0.0) / -60.0;
    SmartDashboard.putNumber("Goal Speed", turningSpeed);
    return turningSpeed;
  }

  public void driveRobotRelative(ChassisSpeeds robotRelativeSpeed) {
    SwerveModuleState[] desiredStates = DriveConstants.kDriveKinematics
        .toSwerveModuleStates(ChassisSpeeds.fromRobotRelativeSpeeds(robotRelativeSpeed, getRotation2d()));
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);

    // Diagnostic: print desired vs measured angle for each module
    System.out.printf("driveRobotRelative: FL desired=%.1fdeg measured=%.1fdeg speed=%.3f\n",
        desiredStates[0].angle.getDegrees(), frontLeft.getPosition().angle.getDegrees(), desiredStates[0].speedMetersPerSecond);
    System.out.printf("driveRobotRelative: FR desired=%.1fdeg measured=%.1fdeg speed=%.3f\n",
        desiredStates[1].angle.getDegrees(), frontRight.getPosition().angle.getDegrees(), desiredStates[1].speedMetersPerSecond);
    System.out.printf("driveRobotRelative: BL desired=%.1fdeg measured=%.1fdeg speed=%.3f\n",
        desiredStates[2].angle.getDegrees(), backLeft.getPosition().angle.getDegrees(), desiredStates[2].speedMetersPerSecond);
    System.out.printf("driveRobotRelative: BR desired=%.1fdeg measured=%.1fdeg speed=%.3f\n",
        desiredStates[3].angle.getDegrees(), backRight.getPosition().angle.getDegrees(), desiredStates[3].speedMetersPerSecond);

    frontLeft.setDesiredState(desiredStates[0]);
    frontRight.setDesiredState(desiredStates[1]);
    backLeft.setDesiredState(desiredStates[2]);
    backRight.setDesiredState(desiredStates[3]);
  }

  public void setModuleStates() {

    SwerveModuleState[] desiredStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);

    // Diagnostic: print desired vs measured angle for each module
    System.out.printf("setModuleStates: FL desired=%.1fdeg measured=%.1fdeg speed=%.3f\n",
        desiredStates[0].angle.getDegrees(), frontLeft.getPosition().angle.getDegrees(), desiredStates[0].speedMetersPerSecond);
    System.out.printf("setModuleStates: FR desired=%.1fdeg measured=%.1fdeg speed=%.3f\n",
        desiredStates[1].angle.getDegrees(), frontRight.getPosition().angle.getDegrees(), desiredStates[1].speedMetersPerSecond);
    System.out.printf("setModuleStates: BL desired=%.1fdeg measured=%.1fdeg speed=%.3f\n",
        desiredStates[2].angle.getDegrees(), backLeft.getPosition().angle.getDegrees(), desiredStates[2].speedMetersPerSecond);
    System.out.printf("setModuleStates: BR desired=%.1fdeg measured=%.1fdeg speed=%.3f\n",
        desiredStates[3].angle.getDegrees(), backRight.getPosition().angle.getDegrees(), desiredStates[3].speedMetersPerSecond);

    frontLeft.setDesiredState(desiredStates[0]);
    frontRight.setDesiredState(desiredStates[1]);
    backLeft.setDesiredState(desiredStates[2]);
    backRight.setDesiredState(desiredStates[3]);

  }

}
