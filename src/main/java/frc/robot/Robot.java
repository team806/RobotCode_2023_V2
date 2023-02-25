// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


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
  
  private final WPI_TalonFX motor_FRmag = new WPI_TalonFX(2);
  private final WPI_TalonFX motor_FLmag = new WPI_TalonFX(4);
  private final WPI_TalonFX motor_RRmag = new WPI_TalonFX(6);
  private final WPI_TalonFX motor_RLmag = new WPI_TalonFX(7);

  //sparks
  private final CANSparkMax motor_FRang = new CANSparkMax(3, MotorType.kBrushless );
  private final CANSparkMax motor_FLang = new CANSparkMax(8, MotorType.kBrushless);
  private final CANSparkMax motor_RRang = new CANSparkMax(1, MotorType.kBrushless);
  private final CANSparkMax motor_RLang = new CANSparkMax(5, MotorType.kBrushless);

  // CANCoders
  CANCoder FR_coder = new CANCoder(9);
  CANCoder FL_coder = new CANCoder(11);
  CANCoder RR_coder = new CANCoder(10);
  CANCoder RL_coder = new CANCoder(12);
  //modules
  //swerveModule FR_module = new swerveModule(motor_FRmag,motor_FRang,FR_coder);
  //swerveModule FL_module = new swerveModule(motor_FLmag,motor_FLang,FL_coder);
  //swerveModule RR_module = new swerveModule(motor_RRmag,motor_RRang,RR_coder);
  //swerveModule RL_module = new swerveModule(motor_RLmag,motor_RLang,RL_coder);
  // encoder position values
  double FR_coderPosition;
  double FL_coderPosition;
  double RR_coderPosition;
  double RL_coderPosition;
  // constants
  protected double swerveRatio = -0.36;
  // motor speed maximums
  protected double angSpeedMax = 0.3;
  protected double magSpeedMax = 1 - (angSpeedMax * -swerveRatio);
  //
  double gyroYaw;
  double CWgyroYaw;
  // modifiers for controller inputs (not for drive or module drive methods)
  double controllerMagMax = 0.8;
  double controllerMagPow = 3;

  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    pid.enableContinuousInput(0, 360);
    FR_coder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
    FR_coder.configSensorDirection(true);
    FR_coder.configMagnetOffset(102+90);
    FL_coder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
    FL_coder.configSensorDirection(true);
    FL_coder.configMagnetOffset(0+90);
    RR_coder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
    RR_coder.configSensorDirection(true);
    RR_coder.configMagnetOffset(-70+90);
    RL_coder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
    RL_coder.configSensorDirection(true);
    RL_coder.configMagnetOffset(-28+90);
    IMU.calibrate();
    IMU.setYawAxis(IMUAxis.kZ);

    SmartDashboard.putNumber("angSpeedMax", angSpeedMax);
    SmartDashboard.putNumber("magSpeedMax", magSpeedMax);

    SmartDashboard.putNumber("controller Mag Max", controllerMagMax);
  }

  @Override
  public void robotPeriodic() {
    // CCW = positive, centered to NORTH
    gyroYaw = IMU.getAngle();
    // CW = positive, centered to EAST
    CWgyroYaw = -(gyroYaw + 90);
    // CW = positive, centered to EAST
    FR_coderPosition = FR_coder.getAbsolutePosition();
    FL_coderPosition = FL_coder.getAbsolutePosition();
    RR_coderPosition = RR_coder.getAbsolutePosition();
    RL_coderPosition = RL_coder.getAbsolutePosition();
    /*
     * SmartDashboard.putNumber("X velocity", velocityX);
     * SmartDashboard.putNumber("Y velocity", velocityY);
     * SmartDashboard.putNumber("X", robotX);
     * SmartDashboard.putNumber("Y", robotY);
     * SmartDashboard.putNumber("Yaw", gyroYaw);
     * SmartDashboard.putNumber("FR codeer ang", FR_coderPosition);
     * SmartDashboard.putNumber("FL codeer ang", FL_coderPosition);
     * SmartDashboard.putNumber("RR codeer ang", RR_coderPosition);
     * SmartDashboard.putNumber("RL codeer ang", RL_coderPosition);
     * 
     * SmartDashboard.putNumber("FR ang motor", motor_FRang.get());
     * SmartDashboard.putNumber("FL ang motor", motor_FLang.get());
     * SmartDashboard.putNumber("RR ang motor", motor_RRang.get());
     * SmartDashboard.putNumber("RL ang motor", motor_RLang.get());
     * SmartDashboard.putNumber("FR mag motor", motor_FRmag.get());
     * SmartDashboard.putNumber("FL mag motor", motor_FLmag.get());
     * SmartDashboard.putNumber("RR mag motor", motor_RRmag.get());
     * SmartDashboard.putNumber("RL mag motor", motor_RLmag.get());
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

    drive(controller.getLeftX(),controller.getLeftY(),Math.pow(controller.getRightX(),controllerMagPow));
    if (controller.getAButtonPressed()) {IMU.reset();}
  }

  /**
   * @return Drives the robot in specified direction, rotates input according to
   *         gyroYaw
   * @param x one to negitive one value for driving left or right respectivly
   * @param y one to negitive one value for driving back or forward respectivly
   * @param z one to negitive one value for driving counter clockwise or clockwise
   *          respectivly
   */
  private void drive(double x, double y, double z) {
    double controlerAng = Math.atan2(y, x) + Math.toRadians(gyroYaw - 90);// rotate by gyro for field
    double controlermag = Math.pow(MathUtil.clamp(Math.hypot(x, y), -1, 1), controllerMagPow) * controllerMagMax;
    x = Math.cos(controlerAng) * controlermag;
    y = Math.sin(controlerAng) * controlermag;

    moduleDrive(motor_FRang, motor_FRmag, FR_coderPosition, x + (z * -0.707106), y + (z * 0.707106));
    moduleDrive(motor_FLang, motor_FLmag, FL_coderPosition, x + (z * 0.707106), y + (z * 0.707106));
    moduleDrive(motor_RRang, motor_RRmag, RR_coderPosition, x + (z * -0.707106), y + (z * -0.707106));
    moduleDrive(motor_RLang, motor_RLmag, RL_coderPosition, x + (z * 0.707106), y + (z * -0.707106));
  }

  /**
   * @return Drive a swerve module acording to an x and y value. NOT field
   *         oriended
   * @param angMotor   the steering motor of the respective module
   * @param magMotor   the steering motor of the respective module
   * @param encoderAng the encoder ang of the respective module
   * @param x          one to negitive one value for driving left or right
   *                   respectivly
   * @param y          one to negitive one value for driving back or forward
   *                   respectivly
   */
  void moduleDrive(CANSparkMax angMotor, WPI_TalonFX magMotor, double encoderAng, double x, double y) {
    double targetAng = Math.toDegrees(Math.atan2(y, x));
    double targetMag = Math.hypot(y, x) / (1 + Math.hypot(0.707106, 0.707106));// scaled to range [-1,1]
    
    angMotor.set(pid.calculate(encoderAng, targetAng) * angSpeedMax);
    magMotor.set((targetMag * magSpeedMax) + (swerveRatio * angMotor.get()));
  }



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
    
  }
}
