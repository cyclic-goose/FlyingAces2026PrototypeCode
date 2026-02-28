package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  private final WPI_TalonSRX feedMotor;
  private final WPI_TalonSRX feedMoveMotor;
  private final TalonFX transferMotor;
  private final TalonFX launchMotor;

  public Shooter(int feedMotorID, int feedMoveMotorID, int transferMotorID, int launchMotorID) {
    // 1. Initialize Feed Motor (Phoenix 5)
    feedMotor = new WPI_TalonSRX(feedMotorID);
    feedMotor.configFactoryDefault();
    feedMotor.setInverted(false);
    feedMotor.configVoltageCompSaturation(12.0);
    feedMotor.enableVoltageCompensation(true);

    // 2. Initialize Feed Move Motor (Phoenix 5)
    // This was missing in your original code
    feedMoveMotor = new WPI_TalonSRX(feedMoveMotorID);
    feedMoveMotor.configFactoryDefault();
    feedMoveMotor.setInverted(false); // Adjust if it goes the wrong way
    feedMoveMotor.configVoltageCompSaturation(12.0);
    feedMoveMotor.enableVoltageCompensation(true);

    // 3. Initialize Transfer and Launch Motors (Phoenix 6)
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Slot0.kP = 0.1;
    config.Slot0.kI = 0.0;
    config.Slot0.kD = 0.0;

    transferMotor = new TalonFX(transferMotorID);
    transferMotor.getConfigurator().apply(config);

    launchMotor = new TalonFX(launchMotorID);
    launchMotor.getConfigurator().apply(config);
  }

  /** Run the Feed Move motor (Bumpers) */
  public void runFeedMove(double speed) {
    feedMoveMotor.set(ControlMode.PercentOutput, speed);
  }

  /** Run the Feed/Intake motor (Left Trigger) */
  public void runFeed(double speed) {
    feedMotor.set(ControlMode.PercentOutput, speed);
  }

  /**
   * * Run the Shooter mechanism (Right Trigger). Usually Launch is faster than Transfer to ensure
   * clean exit.
   */
  public void runShooter(double launchSpeed, double transferSpeed) {
    launchMotor.set(launchSpeed);
    transferMotor.set(transferSpeed);
  }

  /** Stop all motors */
  public void stop() {
    feedMotor.set(ControlMode.PercentOutput, 0);
    feedMoveMotor.set(ControlMode.PercentOutput, 0);
    transferMotor.stopMotor();
    launchMotor.stopMotor();
  }
}
