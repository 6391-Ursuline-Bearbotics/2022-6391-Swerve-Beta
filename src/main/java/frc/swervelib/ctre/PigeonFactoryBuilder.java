package frc.swervelib.ctre;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.swervelib.Gyroscope;

public class PigeonFactoryBuilder {
    public Gyroscope build(PigeonIMU pigeon) {
        return new GyroscopeImplementation(pigeon);
    }

    private static class GyroscopeImplementation implements Gyroscope {
        private final PigeonIMU pigeon;

        private GyroscopeImplementation(PigeonIMU pigeon) {
            this.pigeon = pigeon;
        }

        @Override
        public Rotation2d getGyroHeading() {
            return Rotation2d.fromDegrees(pigeon.getFusedHeading());
        }

        @Override
        public void zeroGyroscope() {
            pigeon.setFusedHeading(0.0);
        }
    }
}
