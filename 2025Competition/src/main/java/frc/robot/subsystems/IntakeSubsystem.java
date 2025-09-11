// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */

  private SparkMax intakeMotor;
  private DigitalInput proximitySensor;

  public IntakeSubsystem() {
    intakeMotor = new SparkMax(IntakeConstants.kIntakeMotorPort, MotorType.kBrushless);
    proximitySensor = new DigitalInput(IntakeConstants.proxSensorPort);
  }

  public void spinMotor(double speed){
    intakeMotor.set(speed);
  }
  public boolean isCoralSensed(){
    return !proximitySensor.get();
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("proximity sensor", proximitySensor.get());
  }
}
