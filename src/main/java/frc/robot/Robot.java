// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  // controller
  private final XboxController controller = new XboxController(0);
  // gyro
  public static final ADIS16470_IMU IMU = new ADIS16470_IMU();
  // PIDs
  PIDController pid = new PIDController(0.02, 0, 0);
  PIDController ballancePID = new PIDController(0.02, 0, 0);
  PIDController driveXPID = new PIDController(0.02, 0, 0);
  PIDController driveYPID = new PIDController(0.02, 0, 0);
  PIDController driveAngPID = new PIDController(0.02, 0, 0);
  // Talons
  private final WPI_TalonFX falcon_FRang = new WPI_TalonFX(3);
  private final WPI_TalonFX falcon_FRmag = new WPI_TalonFX(2);
  private final WPI_TalonFX falcon_FLang = new WPI_TalonFX(8);
  private final WPI_TalonFX falcon_FLmag = new WPI_TalonFX(4);
  private final WPI_TalonFX falcon_RRang = new WPI_TalonFX(1);
  private final WPI_TalonFX falcon_RRmag = new WPI_TalonFX(6);
  private final WPI_TalonFX falcon_RLang = new WPI_TalonFX(5);
  private final WPI_TalonFX falcon_RLmag = new WPI_TalonFX(7);  
  //motors 
  private final motor motor_FRang = new motor(falcon_FRang);
  private final motor motor_FRmag = new motor(falcon_FRmag);
  private final motor motor_FLang = new motor(falcon_FLang);
  private final motor motor_FLmag = new motor(falcon_FLmag);
  private final motor motor_RRang = new motor(falcon_RRang);
  private final motor motor_RRmag = new motor(falcon_RRmag);
  private final motor motor_RLang = new motor(falcon_RLang);
  private final motor motor_RLmag = new motor(falcon_RLmag);
  // CANCoders
  CANCoder FR_coder = new CANCoder(9);
  CANCoder FL_coder = new CANCoder(11);
  CANCoder RR_coder = new CANCoder(10);
  CANCoder RL_coder = new CANCoder(12);
  //modules
  
  //swerveModule FR_module = new swerveModule(motor_FRang,motor_FRmag,FR_coder);
  //swerveModule FL_module = new swerveModule(motor_FLang,motor_FLmag,FL_coder);
  //swerveModule RR_module = new swerveModule(motor_RRang,motor_RRmag,RR_coder);
  //swerveModule RL_module = new swerveModule(motor_RLang,motor_RLmag,RL_coder);
  
  /*
  SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
    new Translation2d(0.381, 0.381),
    new Translation2d(0.381, -0.381),
    new Translation2d(-0.381, -0.381),
    new Translation2d(-0.381, 0.381)
  );
  //FR,FL,RR,RL
  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
    m_kinematics, 
    new Rotation2d(Math.toRadians(IMU.getAngle())),
    new SwerveModulePosition[] {FR_module.getPosition(),FL_module.getPosition(),RR_module.getPosition(),RL_module.getPosition()}, 
    new Pose2d(0, 0,new Rotation2d())
  );
  */

  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    pid.enableContinuousInput(0, 360);
    FR_coder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
    FR_coder.configSensorDirection(true);
    FR_coder.configMagnetOffset(102);
    FL_coder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
    FL_coder.configSensorDirection(true);
    FL_coder.configMagnetOffset(0);
    RR_coder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
    RR_coder.configSensorDirection(true);
    RR_coder.configMagnetOffset(-70);
    RL_coder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
    RL_coder.configSensorDirection(true);
    RL_coder.configMagnetOffset(-28);
    IMU.calibrate();
    IMU.setYawAxis(IMUAxis.kZ);
  }

  @Override
  public void robotPeriodic() {
  /* 
  m_odometry.update(
      new Rotation2d(Math.toRadians(IMU.getAngle())),
      new SwerveModulePosition[] {FR_module.getPosition(),FL_module.getPosition(),RR_module.getPosition(),RL_module.getPosition()}
  );
  SmartDashboard.putNumber("x", m_odometry.getPoseMeters().getX());
  SmartDashboard.putNumber("x", m_odometry.getPoseMeters().getY());
  */
  }
   
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);

  }

  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  @Override
  public void teleopInit() {

  }

  @Override
  public void teleopPeriodic() {
    /* 
    drive(
      new ChassisSpeeds(
        controller.getLeftX(), 
        controller.getLeftY(), 
        controller.getRightX()
      )
    );

    if (controller.getAButtonPressed()) {IMU.reset();}
    */
  }

  /* 
  private void drive(ChassisSpeeds inputSpeeds) {
    SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(inputSpeeds);
    FR_module.setModule(SwerveModuleState.optimize(moduleStates[1],new Rotation2d(FR_coder.getAbsolutePosition())));
    FL_module.setModule(SwerveModuleState.optimize(moduleStates[2],new Rotation2d(FL_coder.getAbsolutePosition())));
    RR_module.setModule(SwerveModuleState.optimize(moduleStates[3],new Rotation2d(RR_coder.getAbsolutePosition())));
    RL_module.setModule(SwerveModuleState.optimize(moduleStates[4],new Rotation2d(RL_coder.getAbsolutePosition())));
  }
*/

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  /* 
    SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(      
      new ChassisSpeeds(
        controller.getLeftX(), 
        controller.getLeftY(), 
        controller.getRightX()
      )
    );

    FR_module.setModule(SwerveModuleState.optimize(moduleStates[1],new Rotation2d(FR_coder.getAbsolutePosition())));
    FL_module.setModule(SwerveModuleState.optimize(moduleStates[2],new Rotation2d(FL_coder.getAbsolutePosition())));
    RR_module.setModule(SwerveModuleState.optimize(moduleStates[3],new Rotation2d(RR_coder.getAbsolutePosition())));
    RL_module.setModule(SwerveModuleState.optimize(moduleStates[4],new Rotation2d(RL_coder.getAbsolutePosition())));
  */
  }
}