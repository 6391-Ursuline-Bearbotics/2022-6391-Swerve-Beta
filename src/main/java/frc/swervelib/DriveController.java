package frc.swervelib;

public interface DriveController {
    void setReferenceVoltage(double voltage);

    double getStateVelocity();
}
