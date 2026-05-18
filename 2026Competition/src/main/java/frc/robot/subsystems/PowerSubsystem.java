package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PowerSubsystem extends SubsystemBase {
    private final PowerDistribution m_pdh = new PowerDistribution(1, ModuleType.kRev);

    public PowerSubsystem() {
        m_pdh.resetTotalEnergy();
    }


@Override
public void periodic() {
    SmartDashboard.putNumber("Total Current", m_pdh.getTotalCurrent());
    SmartDashboard.putNumber("Channel 5 Current", m_pdh.getCurrent(5));
    SmartDashboard.putNumber("Battery Voltage", m_pdh.getVoltage());
}
}
