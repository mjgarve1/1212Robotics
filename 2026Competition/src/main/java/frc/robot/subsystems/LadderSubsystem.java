// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LadderConstants;


public class LadderSubsystem extends SubsystemBase {

  private final SparkMax liftMotor;  
  private Encoder liftEncoder;
  private static double offset;
  public LadderSubsystem() {
    liftMotor = new SparkMax(LadderConstants.kLiftMotorPort, MotorType.kBrushless);

    liftEncoder =  new Encoder(1, 2);
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

  public void resetEncoder(){
    setLiftEncoder(0);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Ladder Height", getLiftEncoder());
    
    SmartDashboard.putNumber("Ladder Offset", getOffset());
    
   // SmartDashboard.putNumber("Ladder Power", liftMotor.getOutputCurrent());
  }
}
