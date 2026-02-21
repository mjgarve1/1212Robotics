// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Rotation;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SwerveJoystickCmd extends Command {
  /** Creates a new SwerveJoystickCmd. */
  private final SwerveSubsystem swerveSubsystem;
  private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
  private final Supplier<Boolean> fieldOrientedFunction, fineDrivingFunction;
  private final Supplier<Boolean> aimAtGoalFunction;
  private PIDController turningPidController;
  private boolean previousFineDrivingState, fineDrivingState;
  private final SlewRateLimiter xLimiter, yLimiter, tLimiter;
  private static final Pose2d GOAL_POSE = new Pose2d(11.9, 4.03, new Rotation2d());

  public SwerveJoystickCmd(SwerveSubsystem swerveSubsystem,
      Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, Supplier<Double> turningSpdFunction,
      Supplier<Boolean> fieldOrientedFunction, Supplier<Boolean> fineDrivingFunction,
      Supplier<Boolean> aimAtGoalFunction) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerveSubsystem = swerveSubsystem;
    this.xSpdFunction = xSpdFunction;
    this.ySpdFunction = ySpdFunction;
    this.turningSpdFunction = turningSpdFunction;
    this.fieldOrientedFunction = fieldOrientedFunction;
    this.fineDrivingFunction = fineDrivingFunction;
    this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
    this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
    this.tLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
    this.aimAtGoalFunction = aimAtGoalFunction;
    previousFineDrivingState = false;
    fineDrivingState = false;
    addRequirements(swerveSubsystem);
    turningPidController = new PIDController(0.5, 0.1, 0);
    turningPidController.enableContinuousInput(-180.0, 180.0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Determine our alliance and change what forward means on the fly
    var invert = 1;
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && alliance.get() == Alliance.Red) {
      invert = -1;
    }

    // 1. Get real-time joystick inputs
    double xSpeed = xSpdFunction.get() * invert;
    xSpeed = xSpeed * -1;

    double ySpeed = ySpdFunction.get() * invert;
    double turningSpeed = turningSpdFunction.get();

    // Toggles fine Driving
    if (fineDrivingFunction.get() && !previousFineDrivingState) {
      fineDrivingState = !fineDrivingState;
      swerveSubsystem.enableReset();
    }
    previousFineDrivingState = fineDrivingFunction.get();

    if (fineDrivingState) {
      turningSpeed /= DriveConstants.kFineTurning;
      xSpeed /= DriveConstants.kFineDriving;
      ySpeed /= DriveConstants.kFineDriving;
    }

    Pose2d robotPose = swerveSubsystem.getPose();
    Rotation2d desiredRotation2d = robotPose.relativeTo(GOAL_POSE).getTranslation().getAngle()
        .minus(Rotation2d.k180deg);
    double testDegrees = desiredRotation2d.getDegrees();
    double offsetDegrees = robotPose.getRotation().minus(desiredRotation2d).getDegrees();

    SmartDashboard.putNumber("Desired Robot Angle", testDegrees);
    SmartDashboard.putNumber("Offset Angle", offsetDegrees);

    if (aimAtGoalFunction.get()) {
      // TEST DRIVING COMMENT
      // Curious if doing .calculate(robotPose.getRotation().getDegrees(),
      // desiredRotation2d.getDegrees()) would be a better approach, worth a try?
      turningSpeed = turningPidController.calculate(offsetDegrees, 0.0) / -60.0;

      SmartDashboard.putNumber("Goal Speed", turningSpeed);
    }

    // 2. Make the driving smoother
    xSpeed = xLimiter.calculate(xSpeed * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond);

    ySpeed = yLimiter.calculate(ySpeed * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond);
    turningSpeed = tLimiter.calculate(turningSpeed) * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;

    // 3. Apply deadband
    // By applying deadband after limiter, we allow a smooth acceleration but
    // immediate stop. This code was payed in blood.
    xSpeed = Math.abs(xSpdFunction.get()) > OIConstants.kControllerAxisDeadband ? xSpeed : 0.0;
    ySpeed = Math.abs(ySpdFunction.get()) > OIConstants.kControllerAxisDeadband ? ySpeed : 0.0;

    if (!aimAtGoalFunction.get()) {
      turningSpeed = Math.abs(turningSpdFunction.get()) > OIConstants.kControllerAxisDeadband ? turningSpeed : 0.0;
    }

    // 4. Construct desired chassis speeds
    ChassisSpeeds chassisSpeeds;
    if (fieldOrientedFunction.get()) {
      // Relative to field
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
          xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d());
    } else {
      // Relative to robot
      chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
    }

    // 6. Output each module states to wheels
    swerveSubsystem.setChassisSpeed(chassisSpeeds);
    swerveSubsystem.setModuleStates();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
