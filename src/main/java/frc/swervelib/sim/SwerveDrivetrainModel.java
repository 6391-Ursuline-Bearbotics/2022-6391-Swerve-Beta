package frc.swervelib.sim;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.PWMSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.wpiClasses.QuadSwerveSim;
import frc.wpiClasses.SwerveModuleSim;
import frc.swervelib.AbsoluteEncoder;
import frc.swervelib.DriveController;
import frc.swervelib.Gyroscope;
import frc.swervelib.Mk4SwerveModuleHelper;
import frc.swervelib.PoseTelemetry;
import frc.swervelib.SteerController;
import frc.swervelib.SwerveModule;

public class SwerveDrivetrainModel {

    QuadSwerveSim swerveDt;
    ArrayList<SwerveModuleSim> modules = new ArrayList<SwerveModuleSim>(QuadSwerveSim.NUM_MODULES);

    ArrayList<SteerController> steerMotorControllers = new ArrayList<SteerController>(QuadSwerveSim.NUM_MODULES);
    ArrayList<DriveController> driveMotorControllers = new ArrayList<DriveController>(QuadSwerveSim.NUM_MODULES);
    ArrayList<AbsoluteEncoder> steerEncoders = new ArrayList<AbsoluteEncoder>(QuadSwerveSim.NUM_MODULES);

    Gyroscope gyro;

    public Field2d field;
    Pose2d endPose;

    private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(Constants.DRIVE.KINEMATICS, gyro.getGyroHeading());

    public SwerveDrivetrainModel(ArrayList<SwerveModule> realModules, Gyroscope gyro){
        this.gyro = gyro;

        if (RobotBase.isSimulation()) {
            modules.add(Mk4SwerveModuleHelper.createSim(realModules.get(0)));
            modules.add(Mk4SwerveModuleHelper.createSim(realModules.get(1)));
            modules.add(Mk4SwerveModuleHelper.createSim(realModules.get(2)));
            modules.add(Mk4SwerveModuleHelper.createSim(realModules.get(3)));

            steerMotorControllers.add(new PWMSim(Constants.FL_steer_MOTOR_IDX));
            steerMotorControllers.add(new PWMSim(Constants.FR_steer_MOTOR_IDX));
            steerMotorControllers.add(new PWMSim(Constants.BL_steer_MOTOR_IDX));
            steerMotorControllers.add(new PWMSim(Constants.BR_steer_MOTOR_IDX));

            driveMotorControllers.add(new PWMSim(Constants.FL_WHEEL_MOTOR_IDX));
            driveMotorControllers.add(new PWMSim(Constants.FR_WHEEL_MOTOR_IDX));
            driveMotorControllers.add(new PWMSim(Constants.BL_WHEEL_MOTOR_IDX));
            driveMotorControllers.add(new PWMSim(Constants.BR_WHEEL_MOTOR_IDX));

            steerEncoders.add(new SimQuadratureEncoder(Constants.FL_steer_ENC_A_IDX, Constants.ENC_PULSE_PER_REV, Constants.steer_ENC_MODULE_REVS_PER_COUNT));
            steerEncoders.add(new SimQuadratureEncoder(Constants.FR_steer_ENC_A_IDX, Constants.ENC_PULSE_PER_REV, Constants.steer_ENC_MODULE_REVS_PER_COUNT));
            steerEncoders.add(new SimQuadratureEncoder(Constants.BL_steer_ENC_A_IDX, Constants.ENC_PULSE_PER_REV, Constants.steer_ENC_MODULE_REVS_PER_COUNT));
            steerEncoders.add(new SimQuadratureEncoder(Constants.BR_steer_ENC_A_IDX, Constants.ENC_PULSE_PER_REV, Constants.steer_ENC_MODULE_REVS_PER_COUNT));

            wheelEncoders.add(new SimQuadratureEncoder(Constants.FL_WHEEL_ENC_A_IDX, Constants.ENC_PULSE_PER_REV, Constants.WHEEL_ENC_WHEEL_REVS_PER_COUNT));
            wheelEncoders.add(new SimQuadratureEncoder(Constants.FR_WHEEL_ENC_A_IDX, Constants.ENC_PULSE_PER_REV, Constants.WHEEL_ENC_WHEEL_REVS_PER_COUNT));
            wheelEncoders.add(new SimQuadratureEncoder(Constants.BL_WHEEL_ENC_A_IDX, Constants.ENC_PULSE_PER_REV, Constants.WHEEL_ENC_WHEEL_REVS_PER_COUNT));
            wheelEncoders.add(new SimQuadratureEncoder(Constants.BR_WHEEL_ENC_A_IDX, Constants.ENC_PULSE_PER_REV, Constants.WHEEL_ENC_WHEEL_REVS_PER_COUNT));
        }
        
        field = PoseTelemetry.field;
        field.setRobotPose(Constants.DFLT_START_POSE);
        endPose = Constants.DFLT_START_POSE;

        swerveDt = new QuadSwerveSim(Constants.DRIVE.TRACKWIDTH_METERS, 
                                     Constants.DRIVE.TRACKWIDTH_METERS, 
                                     Constants.ROBOT.MASS_kg, 
                                     Constants.ROBOT.MOI_KGM2, 
                                     modules);
    }

    /**
     * Handles discontinuous jumps in robot pose. Used at the start of
     * autonomous, if the user manually drags the robot across the field in the
     * Field2d widget, or something similar to that.
     * @param pose
     */
    public void modelReset(Pose2d pose){
        field.setRobotPose(pose);
        swerveDt.modelReset(pose);
        resetPose(pose);
    }

    /**
     * Advance the simulation forward by one step
     * @param isDisabled
     * @param batteryVoltage
     */
    public void update(boolean isDisabled, double batteryVoltage){
 
        // Check if the user moved the robot with the Field2D
        // widget, and reset the model if so.
        Pose2d startPose = field.getRobotPose();
        Transform2d deltaPose = startPose.minus(endPose);
        if(deltaPose.getRotation().getDegrees() > 0.01 || deltaPose.getTranslation().getNorm() > 0.01){
            modelReset(startPose);
        }

        // Calculate and update input voltages to each motor.
        if(isDisabled){
            for(int idx = 0; idx < QuadSwerveSim.NUM_MODULES; idx++){
                modules.get(idx).setInputVoltages(0.0, 0.0);
            }
        } else {
            for(int idx = 0; idx < QuadSwerveSim.NUM_MODULES; idx++){
                steerMotorControllers.setBusVoltage()
                double steerVolts = steerMotorControllers.get(idx).getSpeed() * batteryVoltage;
                double wheelVolts = driveMotorControllers.get(idx).getSpeed() * batteryVoltage;
                modules.get(idx).setInputVoltages(wheelVolts, steerVolts);
            }
        }

        //Update Odometry
        odometry.update(Rotation2d.fromDegrees(gyroscope.getFusedHeading()),
                new SwerveModuleState(frontLeftModule.getDriveVelocity(), new Rotation2d(frontLeftModule.getSteerAngle())),
                new SwerveModuleState(frontRightModule.getDriveVelocity(), new Rotation2d(frontRightModule.getSteerAngle())),
                new SwerveModuleState(backLeftModule.getDriveVelocity(), new Rotation2d(backLeftModule.getSteerAngle())),
                new SwerveModuleState(backRightModule.getDriveVelocity(), new Rotation2d(backRightModule.getSteerAngle()))
        );


        //Update the main drivetrain plant model
        swerveDt.update(Constants.SIM_SAMPLE_RATE_SEC);
        endPose = swerveDt.getCurPose();

        // Update each encoder
        for(int idx = 0; idx < QuadSwerveSim.NUM_MODULES; idx++){
            double steerPos = modules.get(idx).getAzimuthEncoderPositionRev();
            double wheelPos = modules.get(idx).getWheelEncoderPositionRev();
            steerEncoders.get(idx).setShaftPositionRev(steerPos, Constants.SIM_SAMPLE_RATE_SEC);
            wheelEncoders.get(idx).setShaftPositionRev(wheelPos, Constants.SIM_SAMPLE_RATE_SEC);
        }

        // Update associated devices based on drivetrain motion
        field.setRobotPose(endPose);
        gyro.update(startPose, endPose);

    }

    public Pose2d getCurActPose(){
        return field.getRobotObject().getPose();
    }

    public void resetPose(Pose2d pose){
        modelReset(pose);
    }

    public void setSwerveVoltage(double voltage) {
        for(int idx = 0; idx < QuadSwerveSim.NUM_MODULES; idx++){

        }
    }

    public void zeroGyroscope() {
        gyro.zeroGyroscope();
    }

    public Rotation2d getGyroscopeRotation() {
        return gyro.getGyroHeading();
    }

    public void updateTelemetry(){
        if(RobotBase.isSimulation()){
            dtPoseView.setActualPose(simModel.getCurActPose());
        }
        dtPoseView.setEstimatedPose(dtpe.getEstPose());
        dtPoseView.setDesiredPose(dt.getCurDesiredPose());

        curBatVoltage = pdp.getVoltage();
        curBatCurDraw = pdp.getTotalCurrent();

        dtPoseView.update(Timer.getFPGATimestamp()*1000);
    }
}