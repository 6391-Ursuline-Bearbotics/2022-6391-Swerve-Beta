
package frc.swervelib.sim;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.swervelib.SimConstants;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMUSimCollection;

import edu.wpi.first.math.VecBuilder;

public class DrivetrainPoseEstimator {

    /* Singleton infrastructure */
    private static DrivetrainPoseEstimator instance;

    public static DrivetrainPoseEstimator getInstance() {
        if (instance == null) {
            instance = new DrivetrainPoseEstimator();
        }
        return instance;
    }

    Pose2d curEstPose = new Pose2d(SimConstants.DFLT_START_POSE.getTranslation(), SimConstants.DFLT_START_POSE.getRotation());

    Pose2d fieldPose = new Pose2d(); // Field-referenced orign

    boolean pointedDownfield = false;

    SwerveDrivePoseEstimator m_poseEstimator;

    double curSpeed = 0;

    private DrivetrainPoseEstimator() {
        // Trustworthiness of the internal model of how motors should be moving
        // Measured in expected standard deviation (meters of position and degrees of
        // rotation)
        var stateStdDevs = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5));

        // Trustworthiness of gyro in radians of standard deviation.
        var localMeasurementStdDevs = VecBuilder.fill(Units.degreesToRadians(0.1));

        // Trustworthiness of the vision system
        // Measured in expected standard deviation (meters of position and degrees of
        // rotation)
        var visionMeasurementStdDevs = VecBuilder.fill(0.01, 0.01, Units.degreesToRadians(0.1));

        m_poseEstimator = new SwerveDrivePoseEstimator(getGyroHeading(), SimConstants.DFLT_START_POSE,
                SimConstants.m_kinematics, stateStdDevs, localMeasurementStdDevs, visionMeasurementStdDevs,
                SimConstants.CTRLS_SAMPLE_RATE_SEC);

        setKnownPose(SimConstants.DFLT_START_POSE);

    }

    /**
     * Snap update the estimator to a known pose.
     * 
     * @param in known pose
     */
    public void setKnownPose(Pose2d in) {
        DrivetrainControl.getInstance().resetWheelEncoders();
        // No need to reset gyro, pose estimator does that.
        m_poseEstimator.resetPosition(in, getGyroHeading());
        updateDownfieldFlag();
        curEstPose = in;
    }

    public Pose2d getEstPose() {
        return curEstPose;
    }

    public void update() {

        // Based on gyro and measured module speeds and positions, estimate where our
        // robot should have moved to.
        SwerveModuleState[] states = DrivetrainControl.getInstance().getModuleActualStates();
        Pose2d prevEstPose = curEstPose;
        curEstPose = m_poseEstimator.update(getGyroHeading(), states[0], states[1], states[2], states[3]);

        // Calculate a "speedometer" velocity in ft/sec
        Transform2d deltaPose = new Transform2d(prevEstPose, curEstPose);
        curSpeed = Units.metersToFeet(deltaPose.getTranslation().getNorm()) / SimConstants.CTRLS_SAMPLE_RATE_SEC;

        updateDownfieldFlag();
    }

    public Rotation2d getGyroHeading() {
        return Rotation2d.fromDegrees(gyro.getFusedHeading());
    }

    public double getSpeedFtpSec() {
        return curSpeed;
    }

    public void updateDownfieldFlag() {
        double curRotDeg = curEstPose.getRotation().getDegrees();
        pointedDownfield = (curRotDeg > -90 && curRotDeg < 90);
    }

}