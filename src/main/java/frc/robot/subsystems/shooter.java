package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  private final WPI_TalonSRX motor;

  public Shooter(int canID) {
    motor = new WPI_TalonSRX(canID);
    motor.configFactoryDefault();
    motor.setInverted(false);
    motor.configVoltageCompSaturation(12.0);
    motor.enableVoltageCompensation(true);
  }

  /** Run shooter at given speed (-1.0 to 1.0) */
  public void run(double speed) {
    motor.set(ControlMode.PercentOutput, speed);
  }

  /** Stop shooter */
  public void stop() {
    motor.set(ControlMode.PercentOutput, 0);
  }
}
