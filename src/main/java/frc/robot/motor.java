
package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.PIDController;


public class motor {

WPI_TalonFX motor;
PIDController pid = new PIDController(0.02, 0, 0);

    public motor(WPI_TalonFX motorInput) {
      motor = motorInput;
    }

    public void set(double speed){
        motor.set(speed);
    }
    public double get(){
        return motor.get();
    }
    /**@return velocity in radians per cycle */
    public double getVelocity(){
        return motor.getSelectedSensorVelocity()*((Math.PI * 2)/(2048 * 5));
    }
    public void setPoint(double target, double position){
        motor.set(pid.calculate(position, target));
    }

}