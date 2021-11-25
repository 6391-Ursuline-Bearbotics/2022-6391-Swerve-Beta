package frc.swervelib;

import java.util.Objects;

import edu.wpi.first.math.system.plant.DCMotor;

/**
 * A swerve module configuration.
 * <p>
 * A configuration represents a unique mechanical configuration of a module. For example, the Swerve Drive Specialties
 * Mk3 swerve module has two configurations, standard and fast, and therefore should have two configurations
 * ({@link SdsModuleConfigurations#MK3_STANDARD} and {@link SdsModuleConfigurations#MK3_FAST} respectively).
 */
public class ModuleConfiguration {
    private final double wheelDiameter;
    private final double driveReduction;
    private final boolean driveInverted;

    private final double steerReduction;
    private final boolean steerInverted;

    private final DCMotor driveMotor;
    private final DCMotor steerMotor;

    /**
     * Creates a new module configuration.
     *
     * @param wheelDiameter  The diameter of the module's wheel in meters.
     * @param driveReduction The overall drive reduction of the module. Multiplying motor rotations by this value
     *                       should result in wheel rotations.
     * @param driveInverted  Whether the drive motor should be inverted. If there is an odd number of gea reductions
     *                       this is typically true.
     * @param steerReduction The overall steer reduction of the module. Multiplying motor rotations by this value
     *                       should result in rotations of the steering pulley.
     * @param steerInverted  Whether the steer motor should be inverted. If there is an odd number of gear reductions
     *                       this is typically true.
     * @param driveMotor     Motor type of the drive motor.
     * @param steerMotor     Motor type of the steer motor.
     */
    public ModuleConfiguration(double wheelDiameter, double driveReduction, boolean driveInverted,
                               double steerReduction, boolean steerInverted, DCMotor driveMotor,
                               DCMotor steerMotor) {
        this.wheelDiameter = wheelDiameter;
        this.driveReduction = driveReduction;
        this.driveInverted = driveInverted;
        this.steerReduction = steerReduction;
        this.steerInverted = steerInverted;
        this.driveMotor = driveMotor;
        this.steerMotor = steerMotor;
    }

    /**
     * Gets the diameter of the wheel in meters.
     */
    public double getWheelDiameter() {
        return wheelDiameter;
    }

    /**
     * Gets the overall reduction of the drive system.
     * <p>
     * If this value is multiplied by drive motor rotations the result would be drive wheel rotations.
     */
    public double getDriveReduction() {
        return driveReduction;
    }

    /**
     * Gets if the drive motor should be inverted.
     */
    public boolean isDriveInverted() {
        return driveInverted;
    }

    /**
     * Gets the overall reduction of the steer system.
     * <p>
     * If this value is multiplied by steering motor rotations the result would be steering pulley rotations.
     */
    public double getSteerReduction() {
        return steerReduction;
    }

    /**
     * Gets if the steering motor should be inverted.
     */
    public boolean isSteerInverted() {
        return steerInverted;
    }

    /**
     * Gets the type of the drive motor.
     */
    public DCMotor getDriveMotor() {
        return driveMotor;
    }

    /**
     * Gets the type of the steer motor.
     */
    public DCMotor getSteerMotor() {
        return steerMotor;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        ModuleConfiguration that = (ModuleConfiguration) o;
        return Double.compare(that.getWheelDiameter(), getWheelDiameter()) == 0 &&
                Double.compare(that.getDriveReduction(), getDriveReduction()) == 0 &&
                isDriveInverted() == that.isDriveInverted() &&
                Double.compare(that.getSteerReduction(), getSteerReduction()) == 0 &&
                isSteerInverted() == that.isSteerInverted();
    }

    @Override
    public int hashCode() {
        return Objects.hash(
                getWheelDiameter(),
                getDriveReduction(),
                isDriveInverted(),
                getSteerReduction(),
                isSteerInverted()
        );
    }

    @Override
    public String toString() {
        return "ModuleConfiguration{" +
                "wheelDiameter=" + wheelDiameter +
                ", driveReduction=" + driveReduction +
                ", driveInverted=" + driveInverted +
                ", steerReduction=" + steerReduction +
                ", steerInverted=" + steerInverted +
                ", driveMotor=" + driveMotor +
                ", steerMotor=" + steerMotor +
                '}';
    }
}
