// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
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
  //public static final ADIS16470_IMU IMU = new ADIS16470_IMU();
  // PIDs
  PIDController pid = new PIDController(0.02, 0, 0);
  PIDController ballancePID = new PIDController(0.02, 0, 0);
  PIDController driveXPID = new PIDController(0.02, 0, 0);
  PIDController driveYPID = new PIDController(0.02, 0, 0);
  PIDController driveAngPID = new PIDController(0.02, 0, 0);

  //Compressor compressor = new Compressor(1, PneumaticsModuleType.CTREPCM); 
  //DoubleSolenoid exampleDoublePCM = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 1, 0);


  SwerveDriveKinematics m_kinematics;
  SwerveDriveOdometry m_odometry;


  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    
    //compressor.enableAnalog(kDefaultPeriod, kDefaultPeriod);

    pid.enableContinuousInput(90, 100);
    /*
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
    */
    //IMU.calibrate();
    //IMU.setYawAxis(IMUAxis.kZ);
    
    //modules
    swerveModule FR_module = new swerveModule(new motor(new WPI_TalonFX(3)),new motor(new WPI_TalonFX(2)),new CANCoder(9));
    swerveModule FL_module = new swerveModule(new motor(new WPI_TalonFX(8)),new motor(new WPI_TalonFX(4)),new CANCoder(11));
    swerveModule RR_module = new swerveModule(new motor(new WPI_TalonFX(1)),new motor(new WPI_TalonFX(6)),new CANCoder(10));
    swerveModule RL_module = new swerveModule(new motor(new WPI_TalonFX(5)),new motor(new WPI_TalonFX(7)),new CANCoder(12));
    

    m_kinematics = new SwerveDriveKinematics(
      new Translation2d(0.3175,0.3175),
      new Translation2d(-0.3175,0.3175),
      new Translation2d(0.3175,-0.3175),
      new Translation2d(-0.3175,-0.3175)
    );
    /*
    SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
      m_kinematics, 
      new Rotation2d(Math.toRadians(IMU.getAngle())),
      new SwerveModulePosition[] {FR_module.getPosition(),FL_module.getPosition(),RR_module.getPosition(),RL_module.getPosition()}, 
      new Pose2d(0, 0,new Rotation2d())
    );
  */
  }

  @Override
  public void robotPeriodic() {
/*
    m_odometry.update(
      new Rotation2d(Math.toRadians(IMU.getAngle())),
      new SwerveModulePosition[] {FR_module.getPosition(),FL_module.getPosition(),RR_module.getPosition(),RL_module.getPosition()}
    );
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

    if (controller.getYButtonPressed()) {IMU.reset();}
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

    //SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(new ChassisSpeeds(controller.getLeftX(),controller.getLeftY(),-controller.getRightX()));
    SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(getSetVelocity(controller));
    
    SmartDashboard.putNumber("FR module ang", -moduleStates[0].angle.getDegrees() -90);
    SmartDashboard.putNumber("FR module speed", moduleStates[0].speedMetersPerSecond);
    SmartDashboard.putNumber("FL module ang", -moduleStates[1].angle.getDegrees() -90);
    SmartDashboard.putNumber("FL module speed", moduleStates[1].speedMetersPerSecond);
    SmartDashboard.putNumber("RR module ang", -moduleStates[2].angle.getDegrees() -90);
    SmartDashboard.putNumber("RR module speed", moduleStates[2].speedMetersPerSecond);
    SmartDashboard.putNumber("RL module ang", -moduleStates[3].angle.getDegrees() -90);
    SmartDashboard.putNumber("RL module speed", moduleStates[3].speedMetersPerSecond);


    /*if (controller.getAButtonPressed()) {
      exampleDoublePCM.set(Value.kForward);
    }else{
      exampleDoublePCM.set(Value.kReverse);
    }
*/
  }
  private ChassisSpeeds getSetVelocity(XboxController controller) {
    double x = controller.getLeftX();
    double y = controller.getLeftY();
    double speed = MathUtil.clamp(Math.pow(Math.hypot(x,y),3),0,1);
    double ang = Math.atan2(y, x);
    return new ChassisSpeeds(Math.cos(ang)*speed,Math.sin(ang)*speed,controller.getRightX());
  }
}