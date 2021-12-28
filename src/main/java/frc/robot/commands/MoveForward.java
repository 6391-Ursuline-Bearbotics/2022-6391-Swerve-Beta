package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AUTO;
import frc.robot.Constants.DRIVE;
import frc.robot.lib.SwerveDriveKinematicsConstraint6391;
import frc.robot.lib.TrajectoryConfig6391;
import frc.robot.lib.TrajectoryGenerator6391;
import frc.robot.subsystems.DrivetrainSubsystem;

public class MoveForward extends SequentialCommandGroup {
  SwerveDriveKinematicsConstraint6391 constraint = new SwerveDriveKinematicsConstraint6391(DRIVE.KINEMATICS, DRIVE.MAX_FWD_REV_SPEED_MPS);
  TrajectoryConfig6391 config =
        new TrajectoryConfig6391(
                AUTO.kMaxSpeedMetersPerSecond,
                AUTO.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DRIVE.KINEMATICS)
            .addConstraint(constraint);
  Trajectory forward =
        TrajectoryGenerator6391.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Dummy interior waypoint
            List.of(new Translation2d(0, 0)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(3, 0, new Rotation2d(0)),
            config);
  public MoveForward(DrivetrainSubsystem m_drive) {
    addCommands(
      m_drive.dt.createCommandForTrajectory(forward, m_drive)
    );
  }
}