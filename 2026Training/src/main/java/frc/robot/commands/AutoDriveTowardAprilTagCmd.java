// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoDriveTowardAprilTagCmd extends Command {
  /** Creates a new AutoDriveTowardAprilTag. */
  SwerveSubsystem swerveSubsystem;
  Pose2d targetPose;
  PathConstraints constraints;
  public AutoDriveTowardAprilTagCmd(SwerveSubsystem swerveSubsystem) {
    this.swerveSubsystem = swerveSubsystem;
    targetPose = LimelightHelpers.getTargetPose3d_RobotSpace("limelight").toPose2d();
    constraints = new PathConstraints(DriveConstants.kPhysicalMaxSpeedMetersPerSecond, 
    DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond,
     DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond, 
     DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
