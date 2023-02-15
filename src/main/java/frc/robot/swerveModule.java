package frc.robot;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class swerveModule extends Robot {

    
    motor driveMotor;
    motor steerMotor;
    CANCoder moduleEncoder;
    double steerToWheelRatio = (1/6.75);
    double driveToWheelRatio = (7/150) * (17/27) * (45/15);
    double WheelRadiusFt = 1 / 3;
 
    
     
    public swerveModule(motor steer,motor drive,CANCoder encoder){
      driveMotor = drive;
      steerMotor = steer;
      moduleEncoder = encoder;
    }
    
    public swerveModule(){

      }
  
   
    void setModule(SwerveModuleState targetState){
        steerMotor.set(pid.calculate(moduleEncoder.getAbsolutePosition(), targetState.angle.getDegrees()));
        driveMotor.set((targetState.speedMetersPerSecond) + (0.36 * steerMotor.get()));
    }

    SwerveModulePosition getPosition(){
        double velocityRadsPerCycle = (driveMotor.getVelocity() * driveToWheelRatio) + (steerMotor.getVelocity() * steerToWheelRatio);
        return new SwerveModulePosition(
            velocityRadsPerCycle * WheelRadiusFt,
            new Rotation2d(Math.toDegrees(moduleEncoder.getAbsolutePosition()))
        );
    } 
    
}