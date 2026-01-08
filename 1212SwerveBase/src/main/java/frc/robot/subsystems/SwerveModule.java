// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule extends SubsystemBase {

  private SparkMax driveMotor;
  private SparkMax turningMotor;
  private RelativeEncoder driveEncoder;
  private RelativeEncoder turningEncoder;
  private CANcoder absoluteEncoder;
  private boolean absoluteEncoderReversed;
  private double absoluteEncoderOffsetRad;
  private int turningMotorId;
  private SparkClosedLoopController drivingPidController;
  private SparkClosedLoopController turningPidController;
  private SparkMaxConfig config;

  public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
                      int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {
      
    driveMotor = new SparkMax(driveMotorId, MotorType.kBrushless);
    turningMotor = new SparkMax(turningMotorId, MotorType.kBrushless);
    driveEncoder = driveMotor.getEncoder();
    turningEncoder = turningMotor.getEncoder();

    config = new SparkMaxConfig();

    // Configure drive motor
    config
      .inverted(driveMotorReversed)
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(50);
    config.encoder
      .positionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter)
      .velocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
    config.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .pid(DriveConstants.kDriveP, DriveConstants.kDriveI, DriveConstants.kDriveD)
      .outputRange(-1, 1);
    driveMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Configure rotation motor
    config
      .inverted(turningMotorReversed)
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(20);
    config.encoder
      .positionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad)
      .velocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);
    config.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .pid(DriveConstants.kTurningP, DriveConstants.kTurningI, DriveConstants.kTurningD)
      .outputRange(-1, 1)
      .positionWrappingEnabled(true)
      .positionWrappingInputRange(0, ModuleConstants.kTurningEncoderRot2Rad);
    turningMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Set up the absolute encoder to be used to reset the turning motor encoder (basically set the turning motor angle to the real angle)
    this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
    this.absoluteEncoderReversed = absoluteEncoderReversed;
    this.absoluteEncoder = new CANcoder(absoluteEncoderId);
    this.turningMotorId = turningMotorId;
    turningPidController = turningMotor.getClosedLoopController();
    drivingPidController = driveMotor.getClosedLoopController();
    
    resetEncoders();
    
  }

  public void setDesiredState(SwerveModuleState state){
    // If it is supposed to be driving close to 0, just full stop
    if( Math.abs(state.speedMetersPerSecond) < 0.001 ){
        state.speedMetersPerSecond = 0;
    }

    // Optimize the angle to the nearest 180, for example:
    // If the wheel is currently facing directly right and the new command is to go directly left
    // Then there is no need to change the angle of the wheel, just reverse the direction
    // of the drive motor.  A backwards right is the same as a forwards left.  This
    // can be applied to all directions, so this will ensure the wheels never have to turn more than 90
    // degress to get to where they need to be
    state.optimize( getState().angle);

    drivingPidController.setReference(state.speedMetersPerSecond, ControlType.kVelocity);
    turningPidController.setReference(state.angle.getRadians(), ControlType.kPosition);
    
    // Write out what the wheels should be targeting
    SmartDashboard.putString("Swerve[" + turningMotorId+"] state", state.toString());
  }

  public double getAbsoluteEncoderRad(){
    //getAbsolutePosition returns value from -0.5 to 0.5 (think cicle again (-0.5 is right next to 0.5))
    double angle = absoluteEncoder.getAbsolutePosition().getValueAsDouble();
    //boom angle is now in radians
    angle *= (2*Math.PI);
    //offsets the angle according to typed constants
    //pro tip: use Pheonix Tuner X to zero the absolute encoders manually instead
    //EVERY TIME THE ABSOLUTE ENCODERS ARE UNPLUGGED THEY NEED TO BE RE-ZEROED
    angle -= absoluteEncoderOffsetRad;
    //if they are reversed turn that john negative
    if(absoluteEncoderReversed){
      return angle*(-1.0);
    }else{
      return angle;
    }
  }

  //gets drive encoder position in meters
  public double getDrivePosition(){
    return driveEncoder.getPosition();
  }

  //gets turning encoder position in radians
  public double getTurningPosition(){
    return getAbsoluteEncoderRad();
  }

  //gets that swerve module position idk
  public SwerveModulePosition getPosition(){
    return new SwerveModulePosition(getDrivePosition(),new Rotation2d(getTurningPosition()));
  }

  //gets drive encoder velocity in m/s
  public double getDriveVelocity(){
    return driveEncoder.getVelocity();
  }
  
  //gets the state of the module 
  public SwerveModuleState getState(){
    //creates a current state from the current drive velocity and turning position
    return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));    
  }

  public void resetEncoders(){
    //zeros drive encoder
    driveEncoder.setPosition(0);
    //sets the relative turning encoder to it's absolute encoder's value in radians 
    //(remember we converted absolute to radians by multiplying by 2pi and 
    //relative to radians by using our gear ratio to create a conversion factor)
    turningEncoder.setPosition(getAbsoluteEncoderRad());     
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
