package frc.swervelib;

public interface SwerveModule {
    double getDriveVelocity();

    double getSteerAngle();

    ModuleConfiguration getModuleConfiguration();

    void set(double driveVoltage, double steerAngle);
}
