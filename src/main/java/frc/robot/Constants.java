// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
    }
}
