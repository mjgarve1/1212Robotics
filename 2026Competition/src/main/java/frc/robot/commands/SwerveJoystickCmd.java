// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Rotation;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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
    private static final Pose2d GOAL_POSE =
      new Pose2d(11.9, 4.03, new Rotation2d());
  public SwerveJoystickCmd(SwerveSubsystem swerveSubsystem,
  Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, Supplier<Double> turningSpdFunction,
  Supplier<Boolean> fieldOrientedFunction, Supplier<Boolean> fineDrivingFunction, Supplier<Boolean> aimAtGoalFunction) {
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
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    // 1. Get real-time joystick inputs
    double xSpeed = xSpdFunction.get();
    double ySpeed = ySpdFunction.get();
    double turningSpeed = turningSpdFunction.get();

    //Toggles fine Driving
    if (fineDrivingFunction.get() &&  !previousFineDrivingState){ 
      fineDrivingState = !fineDrivingState;
    }
    previousFineDrivingState = fineDrivingFunction.get();

    if(fineDrivingState){
      turningSpeed /= DriveConstants.kFineTurning;
      xSpeed /= DriveConstants.kFineDriving;
      ySpeed /= DriveConstants.kFineDriving;
      swerveSubsystem.enableReset();
    }
    Pose2d robotPose = swerveSubsystem.getPose();

    Rotation2d desiredRotation2d = robotPose.relativeTo(GOAL_POSE).getTranslation().getAngle().minus(Rotation2d.k180deg);

    double testDegrees = desiredRotation2d.getDegrees();
    double offsetDegrees = robotPose.getRotation().minus(desiredRotation2d).getDegrees();

    SmartDashboard.putNumber("Desired Robot Angle", testDegrees);
    SmartDashboard.putNumber("Offset Angle", offsetDegrees);

    if (aimAtGoalFunction.get()) {
    


      double dx = GOAL_POSE.getX() - robotPose.getX();
      double dy = GOAL_POSE.getY() - robotPose.getY();

      Rotation2d targetRotation = new Rotation2d();
      /* 
      if (dx < 0 && dy > 0) {
        targetRotation = new Rotation2d(Math.abs(Math.atan(dx/dy)) + Math.PI/2); }
      else if (dx > 0 && dy > 0) {
        targetRotation = new Rotation2d((3/2)*Math.PI - Math.atan2(dy, dx)); }
      else if (dx > 0 && dy < 0) {
        targetRotation = new Rotation2d(Math.abs(Math.atan2(dy, dx)) + (3/2)*Math.PI);
      }
      else if (dx < 0 && dy < 0){
        targetRotation = new Rotation2d(Math.PI/2 - Math.atan2(dy, dx));
      } 
*/
      if (dx < 0 && dy > 0) {
        targetRotation = new Rotation2d(Math.abs(Math.atan(dy/dx))); }
      else if (dx > 0 && dy > 0) {
        targetRotation = new Rotation2d(Math.abs(Math.atan(dx/dy)) + (Math.PI/2)); }
      else if (dx > 0 && dy < 0) {
        targetRotation = new Rotation2d((3/2 * Math.PI) - Math.abs(Math.atan(dx/dy)));
      }
      else if (dx < 0 && dy < 0){
        targetRotation = new Rotation2d((3/2 * Math.PI) + Math.abs(Math.atan(dy/dx)));
      } 

      Rotation2d currentRotation = robotPose.getRotation();
 

      Rotation2d rotationError = targetRotation.minus(currentRotation);
      //Put a negative here, started working, removed it
      //based on limelight setup, changed black level offset from 5 to 0, and sensor gain from 9.2 to 15
      double degreesError = (rotationError.getDegrees());
      degreesError = offsetDegrees;
      //if rotation error large, move faster, if rotation error small, move slower
      double deadzone = 5;
      double fastzone = 30;
      double mediumzone = 15;
      if (Math.abs(degreesError) > fastzone){
        turningSpeed = 0.8;
        }
      else if (Math.abs(degreesError) <= fastzone && Math.abs(degreesError) > mediumzone){
        turningSpeed = 0.6;
        }
      else if (Math.abs(degreesError) < mediumzone && Math.abs(degreesError) > deadzone){
        turningSpeed = 0.5;
        }
      else {
        turningSpeed = 0;
      }  
      //set proper sign for rotation
      
      if (/*rotationError.getDegrees() */ degreesError > 0 ) {// && rotationError.getDegrees() < 180){
        turningSpeed = turningSpeed;
      }
      else {
        turningSpeed = -turningSpeed;
      } 
      //Turned down for testing
      //turningSpeed = turningSpeed * 0.1;
      double testTurningSpeed = turningPidController.calculate(degreesError, 0.0) / -60.0;
      SmartDashboard.putNumber("Goal Speed", testTurningSpeed);
      turningSpeed = testTurningSpeed;

      SmartDashboard.putNumber("GoalX", GOAL_POSE.getX());
      SmartDashboard.putNumber("GoalY", GOAL_POSE.getY());
      SmartDashboard.putNumber("RobotX", robotPose.getX());
      SmartDashboard.putNumber("RobotY", robotPose.getY());
      SmartDashboard.putNumber("dy", dy);
      SmartDashboard.putNumber("dx", dx);
      SmartDashboard.putNumber("currentRotation", currentRotation.getDegrees());
      SmartDashboard.putNumber("roterror", rotationError.getDegrees());
      SmartDashboard.putNumber("target", targetRotation.getDegrees());
      SmartDashboard.putNumber("arctan", Math.atan2(dx, dy) * (180 / Math.PI));

      // Simple P control
    }
    
    // 2. Make the driving smoother
    xSpeed = xLimiter.calculate(xSpeed * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond);
    
    ySpeed = yLimiter.calculate(ySpeed * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond);
    turningSpeed = tLimiter.calculate(turningSpeed) * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;

    // 3. Apply deadband
    // By applying deadband after limiter, we allow a smooth acceleration but immediate stop. This code was payed in blood.
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
