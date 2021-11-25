// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.UA6391.XboxController6391;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.subsystems.DrivetrainSubsystem;

public class RobotContainer {
  private static final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();

  private static final XboxController6391 m_controller = new XboxController6391(0, 0.05);

  private static final SendableChooser<Command> autoChooser = new SendableChooser<>();

  public RobotContainer() {
    // Set up the default command for the drivetrain.
    // The controls are defined in the command but use both sticks and the triggers on the driver controller
    m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
            m_drivetrainSubsystem,
            m_controller
    ));

    // Populate Auto Chooser
    

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Back button zeros the gyroscope
    // No requirements because we don't need to interrupt anything
    m_controller.BackButton.whenPressed(m_drivetrainSubsystem::zeroGyroscope);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new InstantCommand();
  }
}
