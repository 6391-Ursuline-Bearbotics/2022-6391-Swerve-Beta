package frc.swervelib.kauailabs;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.swervelib.Gyroscope;

public class navXFactoryBuilder {
    public Gyroscope build(AHRS navX) {
        return new GyroscopeImplementation(navX);
    }

    private static class GyroscopeImplementation implements Gyroscope {
        private final AHRS navX;

        private GyroscopeImplementation(AHRS navX) {
            this.navX = navX;
        }

        @Override
        public Rotation2d getGyroHeading() {
            if (navX.isMagnetometerCalibrated()) {
               // We will only get valid fused headings if the magnetometer is calibrated
               return Rotation2d.fromDegrees(navX.getFusedHeading());
            }
            // We have to invert the angle of the NavX so that rotating the robot counter-clockwise makes the angle increase.
            return Rotation2d.fromDegrees(360.0 - navX.getYaw());
        }

        @Override
        public void zeroGyroscope() {
            navX.zeroYaw();
        }
    }
}
