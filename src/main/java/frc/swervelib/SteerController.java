package frc.swervelib;

import edu.wpi.first.math.system.plant.DCMotor;

public interface SteerController {
    double getReferenceAngle();

    void setReferenceAngle(double referenceAngleRadians);

    DCMotor getSteerMotor();

    double getStateAngle();
}
