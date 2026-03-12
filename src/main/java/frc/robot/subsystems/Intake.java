package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private final WPI_TalonSRX feedMotor;
  private final WPI_TalonSRX feedMoveMotor;

  private final DigitalInput limitSwitchBack = new DigitalInput(1);
  private final DigitalInput limitSwitchFront = new DigitalInput(0);

  public Intake(int feedMotorID, int feedMoveMotorID) {
    feedMotor = new WPI_TalonSRX(feedMotorID);
    feedMotor.configFactoryDefault();
    feedMotor.setInverted(false);
    feedMotor.configVoltageCompSaturation(12.0);
    feedMotor.enableVoltageCompensation(true);

    feedMoveMotor = new WPI_TalonSRX(feedMoveMotorID);
    feedMoveMotor.configFactoryDefault();
    feedMoveMotor.setInverted(false);
    feedMoveMotor.configVoltageCompSaturation(12.0);
    feedMoveMotor.enableVoltageCompensation(true);
  }

  public void runFeedMove(double speed) {
    feedMoveMotor.set(ControlMode.PercentOutput, speed);
  }

  public void runFeed(double speed) {
    feedMotor.set(ControlMode.PercentOutput, speed);
  }

  public boolean isFeedLimitBackPressed() {
    return !limitSwitchBack.get();
  }

  public boolean isFeedLimitFrontPressed() {
    return !limitSwitchFront.get();
  }

  public void stop() {
    feedMotor.set(ControlMode.PercentOutput, 0);
    feedMoveMotor.set(ControlMode.PercentOutput, 0);
  }
}
