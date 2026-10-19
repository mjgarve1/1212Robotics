package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.BeltConstants;
import frc.robot.Constants.HerderConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.BeltSubsystem;
import frc.robot.subsystems.HerderSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class RightAutoCmd extends Command {
  private final SwerveSubsystem swerve; 
  private final ShooterSubsystem shooter;
  private final BeltSubsystem belt;
  private boolean shootNow = false;
  private final Timer timer = new Timer();  
  
  public RightAutoCmd(SwerveSubsystem swerve, BeltSubsystem belt, ShooterSubsystem shooter) {
    this.swerve = swerve;
    this.belt = belt;
    this.shooter = shooter;
    addRequirements(swerve, belt, shooter);
  }

  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    shootNow = false;
  }

  @Override
  public void execute() {
    ChassisSpeeds speeds;
    double invert = -1.0;

    if (!timer.hasElapsed(1.0)&& !shootNow) {
      var alliance = DriverStation.getAlliance();
      if (alliance.isPresent() && alliance.get() == Alliance.Red) {
        speeds = new ChassisSpeeds(0.0, 0.0, 0.0);
        belt.setSpeed(0);
        shooter.setSpeed(ShooterConstants.kShooterMotorSpeedAuto);
      } else {
        speeds = new ChassisSpeeds(0.0, 0.0, 0.0);
        belt.setSpeed(0);
        shooter.setSpeed(ShooterConstants.kShooterMotorSpeedAuto);
      }
      swerve.setChassisSpeed(speeds);
      swerve.setModuleStates();
    } else {
      swerve.driveRobotRelative(new ChassisSpeeds(0, 0, 0));
      shootNow = true;

    }
    if (shootNow) {
        belt.setSpeed(BeltConstants.kBeltInSpeed);
        shooter.setSpeed(ShooterConstants.kShooterMotorSpeedAuto);
    }
  }

  @Override
  public boolean isFinished() {
    return false; 
  }




}



