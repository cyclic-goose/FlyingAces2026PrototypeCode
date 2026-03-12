package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  private final TalonFX transferMotor;
  private final TalonFX launchMotor;

  public Shooter(int transferMotorID, int launchMotorID) {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Slot0.kP = 0.1;
    config.Slot0.kI = 0.0;
    config.Slot0.kD = 0.0;

    transferMotor = new TalonFX(transferMotorID);
    transferMotor.getConfigurator().apply(config);

    launchMotor = new TalonFX(launchMotorID);
    launchMotor.getConfigurator().apply(config);
  }

  public void runShooter(double launchSpeed) {
    launchMotor.set(launchSpeed);
  }

  public void runTransfer(double transferSpeed) {
    transferMotor.set(transferSpeed);
  }

  public void stop() {
    transferMotor.stopMotor();
    launchMotor.stopMotor();
  }
}
