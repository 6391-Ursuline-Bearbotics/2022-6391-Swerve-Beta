// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMUSimCollection;
import frc.swervelib.Mk4SwerveModuleHelper;
import frc.swervelib.SdsModuleConfigurations;
import frc.swervelib.SwerveModule;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.PoseTelemetry;
import frc.sim.SwerveRobotModel;
import frc.sim.wpiClasses.QuadSwerveSim;
import frc.sim.wpiClasses.SwerveModuleSim;

import static frc.robot.Constants.*;

public class DrivetrainSubsystem extends SubsystemBase {
  /**
   * The maximum voltage that will be delivered to the drive motors.
   * <p>
   * This can be reduced to cap the robot's maximum speed. Typically, this is useful during initial testing of the robot.
   */
  public static final double MAX_VOLTAGE = 12.0;
  // FIXME Measure the drivetrain's maximum velocity or calculate the theoretical.
  //  The formula for calculating the theoretical maximum velocity is:
  //   <Motor free speed RPM> / 60 * <Drive reduction> * <Wheel diameter meters> * pi
  //  By default this value is setup for a Mk3 standard module using Falcon500s to drive.
  //  An example of this constant for a Mk4 L2 module with NEOs to drive is:
  //   5880.0 / 60.0 / SdsModuleConfigurations.MK4_L2.getDriveReduction() * SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI
  /**
   * The maximum velocity of the robot in meters per second.
   * <p>
   * This is a measure of how fast the robot should be able to drive in a straight line.
   */
  public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 *
          SdsModuleConfigurations.MK4_L1.getDriveReduction() *
          SdsModuleConfigurations.MK4_L1.getWheelDiameter() * Math.PI;
  /**
   * The maximum angular velocity of the robot in radians per second.
   * <p>
   * This is a measure of how fast the robot can rotate in place.
   */
  // Here we calculate the theoretical maximum angular velocity. You can also replace this with a measured amount.
  public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
          Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);

  private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
          // Front left
          new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Front right
          new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Back left
          new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Back right
          new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0)
  );

  // By default we use a Pigeon for our gyroscope. But if you use another gyroscope, like a NavX, you can change this.
  // The important thing about how you configure your gyroscope is that rotating the robot counter-clockwise should
  // cause the angle reading to increase until it wraps back over to zero.
  // FIXME Remove if you are using a Pigeon
  private final PigeonIMU m_pigeon = new PigeonIMU(DRIVETRAIN_PIGEON_ID);
  PigeonIMUSimCollection m_pigeonSim = m_pigeon.getSimCollection();
  // FIXME Uncomment if you are using a NavX
  //  private final AHRS m_navx = new AHRS(SPI.Port.kMXP, (byte) 200); // NavX connected over MXP

  // These are our modules. We initialize them in the constructor.
  private final SwerveModule m_frontLeftModule;
  private final SwerveModule m_frontRightModule;
  private final SwerveModule m_backLeftModule;
  private final SwerveModule m_backRightModule;

  private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

  // Simulation declarations
  private final SwerveModuleSim m_frontLeftModuleSim;
  private final SwerveModuleSim m_frontRightModuleSim;
  private final SwerveModuleSim m_backLeftModuleSim;
  private final SwerveModuleSim m_backRightModuleSim;
  
  SwerveRobotModel m_swerveSimulator;
  private Field2d m_fieldSim;
  DrivetrainControl dt;
  DrivetrainPoseEstimator dtpe;
  PoseTelemetry dtPoseView;
  DriverInterface di;
  PowerDistribution pdp;
  Autonomous a;
  int loopCounter = 0;
  double curBatVoltage = 0;
  double curBatCurDraw = 0;

  public DrivetrainSubsystem() {
    ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

    // There are 4 methods you can call to create your swerve modules.
    // The method you use depends on what motors you are using.
    //
    // Mk3SwerveModuleHelper.createFalcon500(...)
    //   Your module has two Falcon 500s on it. One for steering and one for driving.
    //
    // Mk3SwerveModuleHelper.createNeo(...)
    //   Your module has two NEOs on it. One for steering and one for driving.
    //
    // Mk3SwerveModuleHelper.createFalcon500Neo(...)
    //   Your module has a Falcon 500 and a NEO on it. The Falcon 500 is for driving and the NEO is for steering.
    //
    // Mk3SwerveModuleHelper.createNeoFalcon500(...)
    //   Your module has a NEO and a Falcon 500 on it. The NEO is for driving and the Falcon 500 is for steering.
    //
    // Similar helpers also exist for Mk4 modules using the Mk4SwerveModuleHelper class.

    // By default we will use Falcon 500s in standard configuration. But if you use a different configuration or motors
    // you MUST change it. If you do not, your code will crash on startup.
    // FIXME Setup motor configuration
    m_frontLeftModule = Mk4SwerveModuleHelper.createFalcon500(
            // This parameter is optional, but will allow you to see the current state of the module on the dashboard.
            tab.getLayout("Front Left Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(0, 0),
            // This can either be STANDARD or FAST depending on your gear configuration
            Mk4SwerveModuleHelper.GearRatio.L1,
            // This is the ID of the drive motor
            FRONT_LEFT_MODULE_DRIVE_MOTOR,
            // This is the ID of the steer motor
            FRONT_LEFT_MODULE_STEER_MOTOR,
            // This is the ID of the steer encoder
            FRONT_LEFT_MODULE_STEER_ENCODER,
            // This is how much the steer encoder is offset from true zero (In our case, zero is facing straight forward)
            FRONT_LEFT_MODULE_STEER_OFFSET
    );

    m_frontLeftModuleSim = new SwerveModuleSim(DCMotor.getFalcon500(1), 
                                              DCMotor.getFalcon500(1), 
                                              DRIVETRAIN_WHEEL_DIAMETER_METERS / 2,
                                              DRIVETRAIN_STEER_GEAR_RATIO,
                                              DRIVETRAIN_DRIVE_GEAR_RATIO,
                                              DRIVETRAIN_STEER_ENCODER_RATIO,
                                              DRIVETRAIN_DRIVE_ENCODER_RATIO,
                                              1.1,
                                              0.8,
                                              ROBOT_MASS_kg * 9.81 / QuadSwerveSim.NUM_MODULES, 
                                              0.01 
                                              );

    // We will do the same for the other modules
    m_frontRightModule = Mk4SwerveModuleHelper.createFalcon500(
            tab.getLayout("Front Right Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(2, 0),
            Mk4SwerveModuleHelper.GearRatio.L1,
            FRONT_RIGHT_MODULE_DRIVE_MOTOR,
            FRONT_RIGHT_MODULE_STEER_MOTOR,
            FRONT_RIGHT_MODULE_STEER_ENCODER,
            FRONT_RIGHT_MODULE_STEER_OFFSET
    );

    m_backLeftModule = Mk4SwerveModuleHelper.createFalcon500(
            tab.getLayout("Back Left Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(4, 0),
            Mk4SwerveModuleHelper.GearRatio.L1,
            BACK_LEFT_MODULE_DRIVE_MOTOR,
            BACK_LEFT_MODULE_STEER_MOTOR,
            BACK_LEFT_MODULE_STEER_ENCODER,
            BACK_LEFT_MODULE_STEER_OFFSET
    );

    m_backRightModule = Mk4SwerveModuleHelper.createFalcon500(
            tab.getLayout("Back Right Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(6, 0),
            Mk4SwerveModuleHelper.GearRatio.L1,
            BACK_RIGHT_MODULE_DRIVE_MOTOR,
            BACK_RIGHT_MODULE_STEER_MOTOR,
            BACK_RIGHT_MODULE_STEER_ENCODER,
            BACK_RIGHT_MODULE_STEER_OFFSET
    );

    // Simulation Setup
    if (RobotBase.isSimulation()) {
      m_swerveSimulator = new SwerveRobotModel();
      // Set initial pose of the robot
      //m_swerveSimulator.reset(a.getStartPose());
    }
  }

  /**
   * Sets the gyroscope angle to zero. This can be used to set the direction the robot is currently facing to the
   * 'forwards' direction.
   */
  public void zeroGyroscope() {
    // FIXME Remove if you are using a Pigeon
    m_pigeon.setFusedHeading(0.0);

    // FIXME Uncomment if you are using a NavX
    //    m_navx.zeroYaw();
  }

  public Rotation2d getGyroscopeRotation() {
    // FIXME Remove if you are using a Pigeon
    return Rotation2d.fromDegrees(m_pigeon.getFusedHeading());

    // FIXME Uncomment if you are using a NavX
    //    if (m_navx.isMagnetometerCalibrated()) {
    //      // We will only get valid fused headings if the magnetometer is calibrated
    //      return Rotation2d.fromDegrees(m_navx.getFusedHeading());
    //    }
    //
    //    // We have to invert the angle of the NavX so that rotating the robot counter-clockwise makes the angle increase.
    //    return Rotation2d.fromDegrees(360.0 - m_navx.getYaw());
  }

  public void drive(ChassisSpeeds chassisSpeeds) {
    m_chassisSpeeds = chassisSpeeds;
  }

  @Override
  public void periodic() {
    SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);
    SwerveDriveKinematics.normalizeWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);

    m_frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[0].angle.getRadians());
    m_frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[1].angle.getRadians());
    m_backLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[2].angle.getRadians());
    m_backRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[3].angle.getRadians());
  }

  @Override
  public void simulationPeriodic() {
    m_pigeonSim.setRawHeading();
    m_fieldSim.setRobotPose(getCurrentPose());
    m_swerveSimulator.update(RobotBase.isDisabled());
  }

  private void updateTelemetry(){
    if(RobotBase.isSimulation()){
      dtPoseView.setActualPose(simModel.getCurActPose());
    }
    dtPoseView.setEstimatedPose(dtpe.getEstPose());
    dtPoseView.setDesiredPose(dt.getCurDesiredPose());

    curBatVoltage = pdp.getVoltage();
    curBatCurDraw = pdp.getTotalCurrent();

    dtPoseView.update(Timer.getFPGATimestamp()*1000);
  }

  private SwerveModuleSim createSwerveSim(SwerveModule m_sdsSwerveModule) {
    return new SwerveModuleSim(DCMotor.getFalcon500(1), 
                              DCMotor.getFalcon500(1), 
                              Units.inchesToMeters(Constants.WHEEL_RADIUS_IN),
                              150.0,
                              8.1,
                              1.0,
                              1.0,
                              1.1,
                              0.8,
                              Constants.ROBOT_MASS_kg * 9.81 / QuadSwerveSim.NUM_MODULES, 
                              0.01 
                              );
  }
}
