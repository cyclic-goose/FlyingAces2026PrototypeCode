package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX; // for talonSRX encoder (phoenix 5)
import com.ctre.phoenix6.configs.TalonFXConfiguration; // for initializing TalonFX motors
import com.ctre.phoenix6.hardware.TalonFX; // for the launch and transfer motors
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  private final WPI_TalonSRX feedMotor;
  private final TalonFX transferMotor;
  private final TalonFX launchMotor;
  //

  public Shooter(int feedMotorID, int transferMotorID, int launchMotorID) {
    // initialize feed motor (talonSRX in phoenix 5 library)
    feedMotor = new WPI_TalonSRX(feedMotorID);
    feedMotor.configFactoryDefault();
    feedMotor.setInverted(false);
    feedMotor.configVoltageCompSaturation(12.0);
    feedMotor.enableVoltageCompensation(true);
    // finish initialize feed motor

    // init transfer and launch motors (TalonFX encoders from Phoenix6 library)
    // will use the same configuration for both... for now
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Slot0.kP = 0.1;
    config.Slot0.kI = 0.0;
    config.Slot0.kD = 0.0;
    transferMotor = new TalonFX(transferMotorID);
    transferMotor.getConfigurator().apply(config);

    // same config for launch motor for now
    launchMotor = new TalonFX(launchMotorID);
    launchMotor.getConfigurator().apply(config);
  }

  /** Run shooter at given speed (-1.0 to 1.0) */
  public void run(double speed) {
    feedMotor.set(ControlMode.PercentOutput, speed);
    transferMotor.set(speed);
    launchMotor.set(speed);
  }

  /** Stop shooter */
  public void stop() {
    feedMotor.set(ControlMode.PercentOutput, 0);
    transferMotor.stopMotor();
    launchMotor.stopMotor();
  }
}
