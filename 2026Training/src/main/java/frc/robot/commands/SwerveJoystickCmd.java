// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SwerveJoystickCmd extends Command {
  /** Creates a new SwerveJoystickCmd. */
  private final SwerveSubsystem swerveSubsystem;
  private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
  private final Supplier<Boolean> fieldOrientedFunction, fineDrivingFunction;
  private boolean previousFineDrivingState, fineDrivingState;
  private final SlewRateLimiter xLimiter, yLimiter, tLimiter;
  public SwerveJoystickCmd(SwerveSubsystem swerveSubsystem,
  Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, Supplier<Double> turningSpdFunction,
  Supplier<Boolean> fieldOrientedFunction, Supplier<Boolean> fineDrivingFunction) {
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
        previousFineDrivingState = false;
        fineDrivingState = false;
        addRequirements(swerveSubsystem);
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
/* 
// potential solution to an unknown problem, probably won't be needed.
    if(DriverStation.getAlliance().equals(Alliance.Blue)){
      xSpeed = -xSpeed;
      ySpeed = -ySpeed;
      turningSpeed = -turningSpeed;
    }
     */

    

    //Toggles fine Driving
    if (fineDrivingFunction.get() &&  !previousFineDrivingState){ 
      fineDrivingState = !fineDrivingState;
    }
    previousFineDrivingState = fineDrivingFunction.get();

    if(fineDrivingState){
      turningSpeed /= DriveConstants.kFineTurning;
      xSpeed /= DriveConstants.kFineDriving;
      ySpeed /= DriveConstants.kFineDriving;
    }
    
    // 2. Make the driving smoother
    xSpeed = xLimiter.calculate(xSpeed * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond);
    
    ySpeed = yLimiter.calculate(ySpeed * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond);
    turningSpeed = tLimiter.calculate(turningSpeed) * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;

    // 3. Apply deadband
    // By applying deadband after limiter, we allow a smooth acceleration but immediate stop. This code was payed in blood.
    xSpeed = Math.abs(xSpdFunction.get()) > OIConstants.kControllerAxisDeadband ? xSpeed : 0.0;
    ySpeed = Math.abs(ySpdFunction.get()) > OIConstants.kControllerAxisDeadband ? ySpeed : 0.0;
    turningSpeed = Math.abs(turningSpdFunction.get()) > OIConstants.kControllerAxisDeadband ? turningSpeed : 0.0;
    

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
