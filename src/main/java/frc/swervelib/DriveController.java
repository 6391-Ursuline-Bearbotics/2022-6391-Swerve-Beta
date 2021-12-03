package frc.swervelib;

import edu.wpi.first.math.system.plant.DCMotor;

public interface DriveController {
    void setReferenceVoltage(double voltage);

    DCMotor getDriveMotor();

    double getStateVelocity();

    double getOutputVoltage();

    void setDriveEncoder(double position, double velocity);
}
