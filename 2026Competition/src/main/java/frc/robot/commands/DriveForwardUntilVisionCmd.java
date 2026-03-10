package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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

/** Drive forward until a valid vision pose estimate is available or timeout (2s) */
public class DriveForwardUntilVisionCmd extends Command {
  private final SwerveSubsystem swerve; 
  private final ShooterSubsystem shooter;
  private final BeltSubsystem belt;
  private final Timer timer = new Timer();
  private boolean gotVision = false;

  public DriveForwardUntilVisionCmd(SwerveSubsystem swerve, BeltSubsystem belt, ShooterSubsystem shooter) {
    this.swerve = swerve;
    this.belt = belt;
    this.shooter = shooter;
    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    gotVision = false;
  }

  @Override
  public void execute() {
    ChassisSpeeds speeds;
    if( !timer.hasElapsed(2.0) && !gotVision)
    {
    // Drive forward at 1 m/s (field-relative handled in subsystem)
    speeds = new ChassisSpeeds(1.0, 0.0, 0.0);
    swerve.setChassisSpeed(speeds);
    swerve.setModuleStates();
    }
    else if (!gotVision)
    {
    speeds = new ChassisSpeeds(0.0, 0.0, 0.0);
    swerve.setChassisSpeed(speeds);
    swerve.setModuleStates();

    }

    // Poll Limelight for a pose estimate (blue frame)
    LimelightHelpers.PoseEstimate est = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
    if (est != null && est.tagCount > 0 && !gotVision) {
      // Acceptable vision estimate found: reset pose estimator to this pose and finish
      swerve.zeroHeading(est.pose.getRotation().getDegrees());
      swerve.resetPose(est.pose);
      gotVision = true;
    }

    var alliance = DriverStation.getAlliance();
    Pose2d goalPose = ShooterConstants.BLUE_GOAL_POSE;
    if (alliance.isPresent() && alliance.get() == Alliance.Red) {
      goalPose = ShooterConstants.RED_GOAL_POSE;
    }
    // If gotVision is true, then you can rotate the robot and use its distance to calculate how to shoot the balls.
    if (gotVision) {
      double distance = swerve.getGoalDistance();
      double turningSpeed = swerve.getAimTurningSpeed(goalPose);
      double GoodDistanceHere = 2.0;
      if (Math.abs(turningSpeed) > 0.1) {
        //If it wants to rotate quite a bit, only rotate
        speeds = new ChassisSpeeds(0.0, 0.0, turningSpeed);
        swerve.setChassisSpeed(speeds);
        swerve.setModuleStates();
      }
      else if (distance > GoodDistanceHere) {
        //If it doesnt need to rotate, drive forward until its in range
        speeds = new ChassisSpeeds(1.0, 0.0, 0.0);
        swerve.setChassisSpeed(speeds);
        swerve.setModuleStates();
      }
      else
      {
        //We are in range and looking at the goal, stop driving, shoot the bucket
        speeds = new ChassisSpeeds(0.0, 0.0, 0.0);
        swerve.setChassisSpeed(speeds);
        swerve.setModuleStates();
        belt.setSpeed(BeltConstants.kBeltInSpeed);
      // calculate shooter speed based on distance to goal
      double distanceToGoal = swerve.getGoalDistance();
      // simple linear relationship between distance and shooter speed (tune as necessary)
      double shooterSpeed = ShooterConstants.kShooterMotorSpeed * (distanceToGoal / ShooterConstants.kMaxGoalDistance);
      if(distanceToGoal > ShooterConstants.kMaxGoalDistance) {
        shooterSpeed = ShooterConstants.kShooterMotorSpeed; // cap at max speed
      }
      else if(distanceToGoal < ShooterConstants.kMinGoalDistance) {
        shooterSpeed = ShooterConstants.kShooterMotorSpeed * 0.1; // minimum speed to prevent jamming
      }
      shooter.setSpeed(shooterSpeed);
      }
      // make speed calculation code and THENN do shooting based on distance
      // swerveSubsystem.getAimTurningSpeed(goalPose); can get you the rotation needed to aim at the goal based on the vision pose estimate
      // swerveSubsystem.getGoalDistance(); can get you the distance to the goal
      // You want to have getAimTurningSpeed to be almost 0 (meaning you are pointing directly at the goal)
      // After that, drive the robot forward until you get to the desired shooting distance
      // Once you are at the desired shooting distance, start shooting!

    }
  }

  @Override
  public void end(boolean interrupted) {

  }

  @Override
  public boolean isFinished() {
    // Finish if we got a vision estimate or 2 seconds have elapsed
    return false;
  }
}
