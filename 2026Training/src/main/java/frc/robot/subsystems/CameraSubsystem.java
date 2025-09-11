// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class CameraSubsystem extends SubsystemBase {
  /** Creates a new CameraSubsystem. */
  NetworkTable table;
  NetworkTableEntry tx, ty, ta, tid;
  double x, y, area;
  int id;
  String llName = "limelight";
  public CameraSubsystem() {
    x = LimelightHelpers.getTX(llName);
    y = LimelightHelpers.getTY(llName);
    area = LimelightHelpers.getTA(llName);
    table = NetworkTableInstance.getDefault().getTable("limelight");
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    ta = table.getEntry("ta");
    tid = table.getEntry("tid");
  }

  public double getX(){return tx.getDouble(0.0);}

  public double getY(){return ty.getDouble(0.0);}

  public double getArea(){return ta.getDouble(0.0);}

  public int getID(){return (int)tid.getDouble(0.0);}


  //Use trig to determine distance from april tag
  //Use meters for heightOfCam pretty please.
  public double getDistance(double heightOfCam, double heightOfTag){
    return (heightOfTag - heightOfCam) / Math.tan(y);
  }

  //Uses trig and getDistance as a hypotenuse to get the x and y distance
  public double getXDistance(double heightOfCam, double heightOfTag){
    return Math.sin(x)*getDistance(heightOfCam, heightOfTag);
  }
  public double getYDistance(double heightOfCam, double heightOfTag){
    return Math.cos(x)*getDistance(heightOfCam, heightOfTag);
  }

  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }
}
