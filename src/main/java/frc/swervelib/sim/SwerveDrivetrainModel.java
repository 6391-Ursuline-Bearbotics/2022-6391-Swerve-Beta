package frc.swervelib.sim;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Constants;
import frc.wpiClasses.QuadSwerveSim;
import frc.wpiClasses.SwerveModuleSim;
import frc.swervelib.AbsoluteEncoder;
import frc.swervelib.DriveController;
import frc.swervelib.Gyroscope;
import frc.swervelib.Mk4SwerveModuleHelper;
import frc.swervelib.PoseTelemetry;
import frc.swervelib.SimConstants;
import frc.swervelib.SteerController;
import frc.swervelib.SwerveModule;

public class SwerveDrivetrainModel {

    QuadSwerveSim swerveDt;
    ArrayList<SwerveModule> realModules = new ArrayList<SwerveModule>(QuadSwerveSim.NUM_MODULES);
    ArrayList<SwerveModuleSim> modules = new ArrayList<SwerveModuleSim>(QuadSwerveSim.NUM_MODULES);

    ArrayList<SteerController> steerMotorControllers = new ArrayList<SteerController>(QuadSwerveSim.NUM_MODULES);
    ArrayList<DriveController> driveMotorControllers = new ArrayList<DriveController>(QuadSwerveSim.NUM_MODULES);
    ArrayList<AbsoluteEncoder> steerEncoders = new ArrayList<AbsoluteEncoder>(QuadSwerveSim.NUM_MODULES);

    Gyroscope gyro;

    Field2d field;
    Pose2d endPose;
    PoseTelemetry dtPoseView;

    public SwerveDrivetrainModel(ArrayList<SwerveModule> realModules, Gyroscope gyro){
        this.gyro = gyro;
        this.realModules = realModules;

        if (RobotBase.isSimulation()) {
            modules.add(Mk4SwerveModuleHelper.createSim(realModules.get(0)));
            modules.add(Mk4SwerveModuleHelper.createSim(realModules.get(1)));
            modules.add(Mk4SwerveModuleHelper.createSim(realModules.get(2)));
            modules.add(Mk4SwerveModuleHelper.createSim(realModules.get(3)));
        }
        
        field = PoseTelemetry.field;
        field.setRobotPose(Constants.ROBOT.DFLT_START_POSE);
        endPose = Constants.ROBOT.DFLT_START_POSE;
        dtPoseView = new PoseTelemetry();

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
                double steerVolts = realModules.get(idx).getSteerController().getOutputVoltage();
                double wheelVolts = realModules.get(idx).getDriveController().getOutputVoltage();
                modules.get(idx).setInputVoltages(wheelVolts, steerVolts);
            }
        }

        //Update the main drivetrain plant model
        swerveDt.update(SimConstants.SIM_SAMPLE_RATE_SEC);
        endPose = swerveDt.getCurPose();

        // Update each encoder
        for(int idx = 0; idx < QuadSwerveSim.NUM_MODULES; idx++){
            double steerPos = modules.get(idx).getAzimuthEncoderPositionRev();
            double wheelPos = modules.get(idx).getWheelEncoderPositionRev();
            double steerVelocity = modules.get(idx).getAzimuthEncoderVelocityRPM();
            double wheelVelocity = modules.get(idx).getWheelEncoderVelocityRPM();
            realModules.get(idx).getAbsoluteEncoder().setAbsoluteEncoder(steerPos, steerVelocity);
            realModules.get(idx).getDriveController().setDriveEncoder(wheelPos, wheelVelocity);
        }

        // Update associated devices based on drivetrain motion
        field.setRobotPose(endPose);
        gyro.setAngle(swerveDt.getCurPose().getRotation().getDegrees());

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
            dtPoseView.setActualPose(getCurActPose());
        }
        //dtPoseView.setEstimatedPose(dtpe.getEstPose());
        //dtPoseView.setDesiredPose(getCurDesiredPose());

        dtPoseView.update(Timer.getFPGATimestamp()*1000);
    }
}