// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class ROBOT {
        public static final double MASS_kg = Units.lbsToKilograms(140);
        public static final double MOI_KGM2 = 1.0/12.0 * ROBOT.MASS_kg * Math.pow((DRIVE.TRACKWIDTH_METERS*1.1),2) * 2; //Model moment of intertia as a square slab slightly bigger than wheelbase with axis through center

        public static final double QUIESCENT_CURRENT_DRAW_A = 2.0; //Misc electronics
        public static final double BATTERY_NOMINAL_VOLTAGE = 13.2; //Nicely charged battery
        public static final double BATTERY_NOMINAL_RESISTANCE = 0.040; //40mOhm - average battery + cabling

        // Assumed starting location of the robot. Auto routines will pick their own location and update this.
        static public final Pose2d DFLT_START_POSE = new Pose2d(Units.feetToMeters(24.0), Units.feetToMeters(10.0), Rotation2d.fromDegrees(180));

        /////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // ROBOT PHYSICAL CONSTANTS
        // Robot physical dimensions and mass quantities.
        static public final double ROBOT_MASS_kg = Units.lbsToKilograms(140);
        static public final double ROBOT_MOI_KGM2 = 1.0/12.0 * ROBOT_MASS_kg * Math.pow((DRIVE.WHEELBASE_METERS*1.1),2) * 2; //Model moment of intertia as a square slab slightly bigger than wheelbase with axis through center
        // Location of vision camera relative to robot center - currently front middle.
        static public final Transform2d robotToCameraTrans = new Transform2d(new Translation2d(DRIVE.WHEELBASE_METERS/2, 0), new Rotation2d(0.0));

        // Vision Camera
        static public final String PHOTON_CAM_NAME = "MainCamera";
    }
    public static final class DRIVE {
        /**
         * The left-to-right distance between the drivetrain wheels
         *
         * Should be measured from center to center.
         */
        public static final double TRACKWIDTH_METERS = 1.0; // FIXME Measure and set trackwidth
        /**
         * The front-to-back distance between the drivetrain wheels.
         *
         * Should be measured from center to center.
         */
        public static final double WHEELBASE_METERS = 1.0; // FIXME Measure and set wheelbase

        public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
                new Translation2d(Constants.DRIVE.TRACKWIDTH_METERS / 2.0, Constants.DRIVE.WHEELBASE_METERS / 2.0),
                new Translation2d(Constants.DRIVE.TRACKWIDTH_METERS / 2.0, -Constants.DRIVE.WHEELBASE_METERS / 2.0),
                new Translation2d(-Constants.DRIVE.TRACKWIDTH_METERS / 2.0, Constants.DRIVE.WHEELBASE_METERS / 2.0),
                new Translation2d(-Constants.DRIVE.TRACKWIDTH_METERS / 2.0, -Constants.DRIVE.WHEELBASE_METERS / 2.0)
        );

        public static final double WHEEL_DIAMETER_METERS = 0.10033;
        public static final double WHEEL_CIRCUMFERENCE_METERS = WHEEL_DIAMETER_METERS * Math.PI;

        public static final double STEER_GEAR_RATIO = 1 / ((15.0 / 32.0) * (10.0 / 60.0));
        public static final double DRIVE_GEAR_RATIO = 1 / ((14.0 / 50.0) * (25.0 / 19.0) * (15.0 / 45.0));

        public static final double STEER_ENCODER_RATIO = 1.0;
        public static final double DRIVE_ENCODER_RATIO = 1 / ((14.0 / 50.0) * (25.0 / 19.0) * (15.0 / 45.0));

        public static final int PIGEON_ID = 0; // FIXME Set Pigeon ID

        public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 0; // FIXME Set front left module drive motor ID
        public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 0; // FIXME Set front left module steer motor ID
        public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 0; // FIXME Set front left steer encoder ID
        public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(0.0); // FIXME Measure and set front left steer offset

        public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 0; // FIXME Set front right drive motor ID
        public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 0; // FIXME Set front right steer motor ID
        public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 0; // FIXME Set front right steer encoder ID
        public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(0.0); // FIXME Measure and set front right steer offset

        public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 0; // FIXME Set back left drive motor ID
        public static final int BACK_LEFT_MODULE_STEER_MOTOR = 0; // FIXME Set back left steer motor ID
        public static final int BACK_LEFT_MODULE_STEER_ENCODER = 0; // FIXME Set back left steer encoder ID
        public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(0.0); // FIXME Measure and set back left steer offset

        public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 0; // FIXME Set back right drive motor ID
        public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 0; // FIXME Set back right steer motor ID
        public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 0; // FIXME Set back right steer encoder ID
        public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(0.0); // FIXME Measure and set back right steer offset        

        // Drivetrain Performance Mechanical limits
        static public final double MAX_FWD_REV_SPEED_MPS = Units.feetToMeters(12.0);
        static public final double MAX_STRAFE_SPEED_MPS = Units.feetToMeters(8.0);
        static public final double MAX_ROTATE_SPEED_RAD_PER_SEC = Units.degreesToRadians(360.0);
        static public final double MAX_TRANSLATE_ACCEL_MPS2 = MAX_FWD_REV_SPEED_MPS/0.25; //0-full time of 0.25 second
        static public final double MAX_ROTATE_ACCEL_RAD_PER_SEC_2 = MAX_ROTATE_SPEED_RAD_PER_SEC/0.25; //0-full time of 0.25 second

        /////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // SENSOR CONSTANTS
        // Sensor-related constants - pulled from datasheets for the sensors and gearboxes
        static public final int ENC_PULSE_PER_REV = 1024;
        static public final int WHEEL_ENC_COUNTS_PER_WHEEL_REV = ENC_PULSE_PER_REV;  //Assume 1-1 gearing for now
        static public final int STEER_ENC_COUNTS_PER_MODULE_REV = 4096; //CANCoder
        static public final double WHEEL_ENC_WHEEL_REVS_PER_COUNT  = 1.0/((double)(WHEEL_ENC_COUNTS_PER_WHEEL_REV));
        static public final double steer_ENC_MODULE_REVS_PER_COUNT = 1.0/((double)(STEER_ENC_COUNTS_PER_MODULE_REV));
    }
}
