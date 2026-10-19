package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;

// Subsystem Imports
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.BeltSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// Command Imports
import frc.robot.commands.Auto.AutoBotAlgae;
import frc.robot.commands.DriveForwardUntilVisionCmd;
import frc.robot.commands.RightAutoCmd;

public class AdvancedRightTrenchAuto {

    // 1. Declare Subsystems & Auto Chooser
    private final SwerveSubsystem m_swerveSubsystem;
    private final BeltSubsystem m_beltSubsystem;
    private final ShooterSubsystem m_shooterSubsystem;

    private final SendableChooser<String> m_chooser = new SendableChooser<>();

    public AdvancedRightTrenchAuto(SwerveSubsystem swerveSubsystem, BeltSubsystem beltSubsystem, ShooterSubsystem shooterSubsystem) {
        this.m_swerveSubsystem = swerveSubsystem;
        this.m_beltSubsystem = beltSubsystem;
        this.m_shooterSubsystem = shooterSubsystem;

        // 2. Register Named Commands for PathPlanner
        // Register any named commands BEFORE building autos or initializing AutoBuilder
        NamedCommands.registerCommand("AutoBotAlgae", new AutoBotAlgae());
        NamedCommands.registerCommand("VisionAcquire", new DriveForwardUntilVisionCmd(m_swerveSubsystem, m_beltSubsystem, m_shooterSubsystem).withTimeout(2.0));
        NamedCommands.registerCommand("Shoot5Point5", new RightAutoCmd(m_swerveSubsystem, m_beltSubsystem, m_shooterSubsystem).withTimeout(5.5));

        // 3. Configure Autonomous Chooser Options
        // Set "Right Auto" from your PathPlanner setup as default
        m_chooser.setDefaultOption("Right Auto", "Right Auto"); 
        m_chooser.addOption("Blue Top Auto", "BlueTopAuto");

        // 4. Publish Chooser to SmartDashboard & Shuffleboard
        SmartDashboard.putData("Auto Choices", m_chooser);

        // Optional: Put onto a dedicated Shuffleboard "Autonomous" tab
        Shuffleboard.getTab("Autonomous")
            .add("Auto Selector", m_chooser)
            .withWidget(BuiltInWidgets.kComboBoxChooser)
            .withPosition(0, 0)
            .withSize(2, 1);

        // Configure default button bindings
        configureButtonBindings();
    }

    private void configureButtonBindings() {
        // Configure controller bindings here
    }

    /**
     * Passes the selected autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // Uses PathPlanner's AutoBuilder to construct the selected auto routine dynamically
        String selectedAuto = m_chooser.getSelected();

        if (selectedAuto == null) {
            return new InstantCommand(); // Return empty command if nothing selected
        }

        Command pathPlannerAuto = AutoBuilder.buildAuto(selectedAuto);

        // Sequence:
        // 1. Move back using Limelight / Vision for 2 seconds
        // 2. Shoot for 5.5 seconds
        // 3. Run the selected PathPlanner auto routine
        return Commands.sequence(
            new DriveForwardUntilVisionCmd(m_swerveSubsystem, m_beltSubsystem, m_shooterSubsystem).withTimeout(2.0),
            new RightAutoCmd(m_swerveSubsystem, m_beltSubsystem, m_shooterSubsystem).withTimeout(5.5),
            pathPlannerAuto
        );
    }
}