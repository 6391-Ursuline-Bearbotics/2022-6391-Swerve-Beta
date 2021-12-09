package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.lib.XboxController6391;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DefaultDriveCommand extends CommandBase {
    private final DrivetrainSubsystem m_drivetrainSubsystem;
    private final XboxController6391 m_controller;
    private static final SendableChooser<String> driverChooser = new SendableChooser<>();
    private static final SendableChooser<String> orientationChooser = new SendableChooser<>();

    private double m_translationX;
    private double m_translationY;
    private double m_rotation;

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
              m_translationX = modifyAxis(-m_controller.JoystickLY());
              m_translationY = modifyAxis(-m_controller.JoystickLX());
              m_rotation = modifyAxis(m_controller.JoystickRX());
              break;
            case "Left Stick and Triggers":
              m_translationX = modifyAxis(-m_controller.JoystickLY());
              m_translationY = modifyAxis(-m_controller.JoystickLX());
              m_rotation = m_controller.TriggerCombined();
              break;
            case "Gas Pedal":
              m_translationX = modifyAxis(-m_controller.JoystickLY());
              m_translationY = modifyAxis(-m_controller.JoystickLX());
              double angle = calculateTranslationDirection(m_translationX, m_translationY);
              m_translationX = Math.cos(angle) * m_controller.TriggerR();
              m_translationY = Math.sin(angle) * m_controller.TriggerR();
              m_rotation = modifyAxis(m_controller.JoystickRX());
              break;
        }

        m_drivetrainSubsystem.dt.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        m_translationX * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
                        m_translationY * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
                        m_rotation * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
                        m_drivetrainSubsystem.dt.getGyroscopeRotation()
                )
        );
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.dt.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }

  private static double modifyAxis(double value) {
      // Square the axis
      value = Math.copySign(value * value, value);

      return value;
  }

  /**
     * Calculates the angle of translation set by the left stick.
     *
     * @return The angle of translation. 0 corresponds to forwards, and positive
     *     corresponds to counterclockwise.
    */
  private double calculateTranslationDirection(double x, double y) {
    // Calculate the angle.
    // Swapping x/y and inverting y because our coordinate system has +x forwards and -y right
    return Math.atan2(x, -y);
  }
}