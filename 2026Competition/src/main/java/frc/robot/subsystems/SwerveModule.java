// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule extends SubsystemBase {
  /** Creates a new SwerveModule. */

  private SparkMax driveMotor;
  private SparkMax turningMotor;

  private RelativeEncoder driveEncoder;
  private RelativeEncoder turningEncoder;

  private CANcoder absoluteEncoder;

  private boolean absoluteEncoderReversed;

  private double absoluteEncoderOffsetRad;
  private int turningMotorId;

  private PIDController turningPidController;

  private SparkMaxConfig config;

  public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
      int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {

    // Initialize the drive and rotation motors
    config = new SparkMaxConfig();
    this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
    this.absoluteEncoderReversed = absoluteEncoderReversed;
    this.absoluteEncoder = new CANcoder(absoluteEncoderId);
    driveMotor = new SparkMax(driveMotorId, MotorType.kBrushless);
    turningMotor = new SparkMax(turningMotorId, MotorType.kBrushless);
    this.turningMotorId = turningMotorId;
    driveEncoder = driveMotor.getEncoder();
    turningEncoder = turningMotor.getEncoder();

    // Configure drive motor
    config
        .inverted(driveMotorReversed) // Set direction
        .idleMode(IdleMode.kBrake); // Set motor to stop when not being commanded to move
    config.encoder
        .positionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter) // How far each rotation of the motor moves
                                                                          // the robot, used to track actual distance
                                                                          // travelled
        .velocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec); // How fast the robot moves when the
                                                                                 // motor is spinning, used to determine
                                                                                 // how fast the robot moves/is moving
    driveMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters); // Write the
                                                                                                  // configuration

    // Configure rotation motor
    config
        .inverted(turningMotorReversed) // Set direction
        .idleMode(IdleMode.kCoast); // Set motor to keep moving when not being actively driven
    config.encoder
        .positionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad) // Convert motor encoder position to angle of
                                                                          // the wheel
        .velocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec); // Convert how fast the angle of the
                                                                                 // wheel is changing
    turningMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // The Rotation Motor is meant to drive to a desired position, not a desired
    // speed
    // Due to this, we need to set up a PID controller that goes from -pi to pi
    // (full circle)
    turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0);
    turningPidController.enableContinuousInput(-Math.PI, Math.PI);

    resetEncoders();

  }

  public void setDesiredState(SwerveModuleState state) {
    // If it is supposed to be driving close to 0, just full stop everything
    if (Math.abs(state.speedMetersPerSecond) < 0.001) {
      stop();
      return;
    }

    // Optimize the angle to the nearest 180, for example:
    // If the wheel is currently facing directly right and the new command is to go
    // directly left
    // Then there is no need to change the angle of the wheel, just reverse the
    // direction
    // of the drive motor. A backwards right is the same as a forwards left. This
    // can be applied to all directions, so this will ensure the wheels never have
    // to turn more than 90
    // degress to get to where they need to be
    state.optimize(getState().angle);

    // Drive the motor at the specified speed. Motors take speeds from -1 to 1 as a
    // double
    // So, if the motors at 100% go at X m/s, to go Y m/s you have to divide Y/X to
    // get the range to -1 to 1.
    driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);

    // Turn the motor to the desired position using the PID defined above using the
    // current wheel position
    // and the desired wheel position. Tuning this will help the robot drive
    // different directions better.
    turningMotor.set(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));

    // Write out what the wheels should be targeting
    SmartDashboard.putString("Swerve[" + turningMotorId + "] state", state.toString());

  }

  // gets drive encoder position in meters
  public double getDrivePosition() {
    return driveEncoder.getPosition();
  }

  // gets turning encoder position in radians
  public double getTurningPosition() {
    return getAbsoluteEncoderRad();
  }

  // gets that swerve module position idk
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getDrivePosition(), new Rotation2d(getTurningPosition()));
  }

  // gets drive encoder velocity in m/s
  public double getDriveVelocity() {
    return driveEncoder.getVelocity();
  }

  // gets turning encoder velocity in rad/s
  public double getTurningVelocity() {
    return turningEncoder.getVelocity();
  }

  public double getAbsoluteEncoderRad() {
    // getAbsolutePosition returns value from -0.5 to 0.5 (think cicle again (-0.5
    // is right next to 0.5))
    double angle = absoluteEncoder.getAbsolutePosition().getValueAsDouble();
    // boom angle is now in radians
    angle *= (2 * Math.PI);
    // offsets the angle according to typed constants
    // pro tip: use Pheonix Tuner X to zero the absolute encoders manually instead
    // EVERY TIME THE ABSOLUTE ENCODERS ARE UNPLUGGED THEY NEED TO BE RE-ZEROED
    angle -= absoluteEncoderOffsetRad;
    // if they are reversed turn that john negative
    if (absoluteEncoderReversed) {
      return angle * (-1.0);
    } else {
      return angle;
    }
  }

  public double getAbsoluteEncoderReading() {
    // literally just returns the method above this one
    // ik its basically useless but im not changing it now
    return getAbsoluteEncoderRad();
  }

  public void resetEncoders() {
    // zeros drive encoder
    driveEncoder.setPosition(0);
    // sets the relative turning encoder to it's absolute encoder's value in radians
    // (remember we converted absolute to radians by multiplying by 2pi and
    // relative to radians by using our gear ratio to create a conversion factor)
    turningEncoder.setPosition(getAbsoluteEncoderRad());

  }

  // gets absolute position of absolute encoders on a range of -0.5 to 0.5 (raw
  // data with no conversions)
  public double getAbsolutePos() {
    return absoluteEncoder.getAbsolutePosition().getValueAsDouble();
  }

  // gets the state of the module
  public SwerveModuleState getState() {
    // creates a current state from the current drive velocity and turning position
    return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
  }

  // stop
  public void stop() {
    driveMotor.set(0);
    turningMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
