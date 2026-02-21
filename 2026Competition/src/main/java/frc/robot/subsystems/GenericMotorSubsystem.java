// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.PersistMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GenericMotorSubsystem extends SubsystemBase {
  //Creates a new Generic Motor Subsystem. 
  private final SparkMax motor;
  private final RelativeEncoder encoder;
  private SparkMaxConfig config;

  public GenericMotorSubsystem(int sparkMaxId, MotorType motorType) {
    motor = new SparkMax(sparkMaxId, motorType);
    encoder = motor.getEncoder();
    
    config = new SparkMaxConfig();

    config.idleMode(IdleMode.kBrake);

    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  public double getEncoderPosition(){
    return encoder.getPosition();
  }

  public void resetEncoderPosition(){
    encoder.setPosition(0);
  }

  public void setSpeed(double speed){
    motor.set(speed);
  }

  @Override
  public void periodic() {

  }
}
