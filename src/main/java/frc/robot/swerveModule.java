package frc.robot;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class swerveModule extends Robot {

    
    WPI_TalonFX driveMotor;
    CANSparkMax steerMotor;
    CANCoder moduleEncoder;
    double driveToWheelRatio = (1/6.75);
    double encoderToWheelRatio = (17/27) * (45/15);
    double WheelRadiusFt = 1 / 3;
 
    
    public swerveModule(){

      }

    public swerveModule(WPI_TalonFX drive, CANSparkMax steer, CANCoder encoder) {
      driveMotor = drive;
      steerMotor = steer;
      moduleEncoder = encoder;
    }

    void setModule(SwerveModuleState targetState){
        steerMotor.set(pid.calculate(moduleEncoder.getAbsolutePosition(), targetState.angle.getDegrees()));
        driveMotor.set((targetState.speedMetersPerSecond) + (0.36 * steerMotor.get()));
    }

    SwerveModulePosition getPosition(){
        double velocityRadsPerCycle = (getFalconSpeed(driveMotor) * driveToWheelRatio) + (getCANcoderSpeed() * encoderToWheelRatio);
        return new SwerveModulePosition(
            velocityRadsPerCycle * WheelRadiusFt,
            new Rotation2d(Math.toRadians(moduleEncoder.getAbsolutePosition()))
        );
    } 
    /**
     * @return returns speed in radians per cycle 
     */
    double getFalconSpeed(WPI_TalonFX motor){
      return motor.getSelectedSensorVelocity() * ((2*Math.PI)/2048) * (1/5);
    }
    /**
     * @return returns speed in radians per cycle 
     */
    double getCANcoderSpeed(){
      return moduleEncoder.getVelocity() * (Math.PI/180) * (1/50);
    }
    
}