// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.ObjectInputFilter.Config;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LadderConstants;


public class LadderSubsystem extends SubsystemBase {
  /** Creates a new LadderSubsystem. */

  //coded as brushless can code as brushed if neccesary.
  //private final SparkMax liftMotor = new SparkMax(LadderConstants.kLiftMotorPort, MotorType.kBrushless);
  //private final RelativeEncoder liftEncoder = liftMotor.getEncoder();
  //
  //Currently brushed, easy af to change - J
  private final SparkMax liftMotor;
  //private final RelativeEncoder liftEncoder;
  
  private Encoder liftEncoder;
  private double lastSetPoint;
  private SparkMaxConfig config;
  private static double offset;
  public LadderSubsystem() {
    liftMotor = new SparkMax(LadderConstants.kLiftMotorPort, MotorType.kBrushless);

    liftEncoder =  new Encoder(1, 2);

    
    //liftEncoder = liftMotor.getAlternateEncoder();
    /* 
    config = new SparkMaxConfig();
    config
      .smartCurrentLimit(40, 40)
      .idleMode(IdleMode.kCoast);
    liftMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);  
   */ lastSetPoint = 0;
   offset = getLiftEncoder();
  }

  public void driveLift(double speed){
    liftMotor.set(speed);
  }

  public double getLiftEncoder(){return liftEncoder.get()/1000.0;}

  public void setLiftEncoder(double val){
    //liftEncoder.setPosition(val);
    
  }
  public static void setOffset(double off){
    offset = off;
  }
  public static double getOffset(){
    return offset;
  }
  public double getLastSetPoint(){return lastSetPoint;}
  public void setLastPoint(double setPoint){
    lastSetPoint = setPoint;
  }

  public void resetEncoder(){
    setLiftEncoder(0);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Height", getLiftEncoder());
    
    SmartDashboard.putNumber("offset", getOffset());
    
    
    SmartDashboard.putNumber("LadderCurrent", liftMotor.getOutputCurrent());
  }
}
