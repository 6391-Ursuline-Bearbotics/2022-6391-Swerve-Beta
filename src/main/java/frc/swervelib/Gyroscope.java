package frc.swervelib;

import edu.wpi.first.math.geometry.Rotation2d;

public interface Gyroscope {
  Rotation2d getGyroHeading();

  /**
  * Sets the gyroscope angle to zero. This can be used to set the direction the robot is currently facing to the
  * 'forwards' direction.
  */
  void zeroGyroscope();
}
