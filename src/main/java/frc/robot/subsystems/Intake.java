package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private final WPI_TalonSRX feedMotor1;
  private final WPI_TalonSRX feedMotor2;
  private final WPI_TalonSRX feedMoveMotor;

  private final DigitalInput limitSwitchBack = new DigitalInput(1);
  private final DigitalInput limitSwitchFront = new DigitalInput(0);

  public Intake(int feedMotor1ID, int feedMotor2ID, int feedMoveMotorID) {
    // configure feed motor 1
    feedMotor1 = new WPI_TalonSRX(feedMotor1ID);
    feedMotor1.configFactoryDefault();
    feedMotor1.setInverted(false);
    feedMotor1.configVoltageCompSaturation(12.0);
    feedMotor1.enableVoltageCompensation(true);

    // configure feed motor 2
    feedMotor2 = new WPI_TalonSRX(feedMotor2ID);
    feedMotor2.configFactoryDefault();
    feedMotor2.setInverted(false);
    feedMotor2.configVoltageCompSaturation(12.0);
    feedMotor2.enableVoltageCompensation(true);

    // configure talon srx for feed movement motor
    feedMoveMotor = new WPI_TalonSRX(feedMoveMotorID);
    feedMoveMotor.configFactoryDefault();
    feedMoveMotor.setInverted(false);
    feedMoveMotor.configVoltageCompSaturation(12.0);
    feedMoveMotor.enableVoltageCompensation(true);
  }

  public void runFeedMove(double speed) {
    feedMoveMotor.set(ControlMode.PercentOutput, speed);
  }

  public void runFeedMovePeriod(double speed) {
    if (!(isFeedLimitBackPressed() && isFeedLimitFrontPressed())) {
      feedMoveMotor.set(ControlMode.PercentOutput, 0.85);
    } else {
      feedMoveMotor.set(ControlMode.PercentOutput, 0.0);
    }
  }

  public void runFeed(double speed) {
    // run both feed motors
    feedMotor1.set(ControlMode.PercentOutput, speed);
    feedMotor2.set(ControlMode.PercentOutput, speed);
  }

  public boolean isFeedLimitBackPressed() {
    return !limitSwitchBack.get();
  }

  public boolean isFeedLimitFrontPressed() {
    return !limitSwitchFront.get();
  }

  public void stop() {
    feedMotor1.set(ControlMode.PercentOutput, 0);
    feedMotor2.set(ControlMode.PercentOutput, 0);
    feedMoveMotor.set(ControlMode.PercentOutput, 0);
  }
}
