package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.PersistMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HerderSubsystem extends SubsystemBase{
    private final SparkMax herderMotor;
    private final RelativeEncoder herderEncoder;
    private final SparkMax winchMotor;
    private final RelativeEncoder winchEncoder;
    private SparkMaxConfig config;

   public HerderSubsystem(int sparkMaxId, int sparkMaxId2) {
    herderMotor = new SparkMax(sparkMaxId, MotorType.kBrushless);
    herderEncoder = herderMotor.getEncoder();
    winchMotor = new SparkMax(sparkMaxId2, MotorType.kBrushless);
    winchEncoder = winchMotor.getEncoder();
    winchEncoder.setPosition(0);
    
    config = new SparkMaxConfig();

    config.idleMode(IdleMode.kBrake);

    herderMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  public void setHerderSpeed(double speed){
    herderMotor.set(speed);
    double herderCurrent = herderMotor.getOutputCurrent();
    SmartDashboard.putNumber("herder current", herderCurrent);
  }

  public void setWinchSpeed(double speed){
    double maxWinchLimit = -52.5; //example, find real limit
    double minWinchLimit = 2.5;
    if (winchEncoder.getPosition() < maxWinchLimit && speed < 0) {
      speed = 0;
    }
    else if (winchEncoder.getPosition() > minWinchLimit && speed > 0) {
      speed = 0;
    }
    //TODO: Determine the encoder position limits for the winch and implement logic to prevent the winch from moving beyond those limits
    winchMotor.set(speed);
    SmartDashboard.putNumber("Winch", winchEncoder.getPosition());
    SmartDashboard.putNumber("Winch Speed", speed);
    double winchCurrent = winchMotor.getOutputCurrent();
    SmartDashboard.putNumber("winch current", winchCurrent);
    }

  public double getWinchEncoderPosition(){
    double position = winchEncoder.getPosition();
    SmartDashboard.putNumber("Winch Encoder", position);
    return position;
  }

 /* public double WinchIn(double speed)

  public void resetEncoderPosition(){
    winchEncoder.setPosition(0);
  }
*/

  @Override
  public void periodic() {

  }
}
