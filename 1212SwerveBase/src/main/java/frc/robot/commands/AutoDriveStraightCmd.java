// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoDriveStraightCmd extends Command {
  /** Creates a new AutoDriveStraightCmd. */
  private SwerveSubsystem swerveSubsystem;
  private double xSpeed, ySpeed, thetaStartPoint;
  private PIDController headingPidController;
  private ChassisSpeeds chassisSpeeds;
  public AutoDriveStraightCmd(SwerveSubsystem swerveSubsystem, double x, double y) {
    this.swerveSubsystem = swerveSubsystem;
     xSpeed = x;
     ySpeed = y;
     thetaStartPoint = swerveSubsystem.getHeadingRadians();
     headingPidController = new PIDController(1.0, 0, 0);
     headingPidController.setSetpoint(this.thetaStartPoint);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    chassisSpeeds = new ChassisSpeeds(xSpeed,ySpeed, headingPidController.calculate(swerveSubsystem.getHeadingRadians()));
    swerveSubsystem.setChassisSpeed(chassisSpeeds);
    swerveSubsystem.setModuleStates();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.driveRobotRelative(new ChassisSpeeds(0,0,0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
