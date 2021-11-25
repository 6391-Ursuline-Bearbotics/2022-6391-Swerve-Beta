package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.UA6391.XboxController6391;
import frc.robot.subsystems.DrivetrainSubsystem;

import java.util.function.DoubleSupplier;

public class DefaultDriveCommand extends CommandBase {
    private final DrivetrainSubsystem m_drivetrainSubsystem;
    private final XboxController6391 m_controller;
    private static final SendableChooser<String> driverChooser = new SendableChooser<>();
    private static final SendableChooser<String> orientationChooser = new SendableChooser<>();

    public DefaultDriveCommand(DrivetrainSubsystem drivetrainSubsystem,
                               XboxController6391 controller) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.m_controller = controller;

        // Control Scheme Chooser
        driverChooser.setDefaultOption("Both Sticks", "Both Sticks");
        driverChooser.addOption("Left Stick and Triggers", "Left Stick and Triggers");
        driverChooser.addOption("Gas Pedal", "Gas Pedal");

        // Control Orientation Chooser
        orientationChooser.setDefaultOption("Field Oriented", "Field Oriented");
        orientationChooser.addOption("Robot Oriented", "Robot Oriented");

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute() {
        switch (driverChooser.getSelected()) {
            case "Both Sticks":
            case "Left Stick and Triggers":
              return -modifyAxis(m_controller.JoystickLX()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND;
            case "Gas Pedal":


        m_drivetrainSubsystem.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        m_translationXSupplier.getAsDouble(),
                        m_translationYSupplier.getAsDouble(),
                        m_rotationSupplier.getAsDouble(),
                        m_drivetrainSubsystem.getGyroscopeRotation()
                )
        );
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
}


() -> translationX(),
() -> -modifyAxis(m_controller.getLeftY()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
() -> -modifyAxis(m_controller.getRightX()) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND