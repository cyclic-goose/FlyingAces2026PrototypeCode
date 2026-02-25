// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class shooter extends SubsystemBase {

  // forward declare motor but dont create it
  private final TalonFX testMotor;

  /** Creates a new shooter. */
  public shooter(int canID) {
    // construct motor with input canID
    testMotor = new TalonFX(canID);
    // set to default config
    testMotor.getConfigurator().apply(new TalonFXConfiguration());
  }

  /** Stop the motor */
  public void stop() {
    testMotor.set(0);
    System.out.println("Shooter motor stopped");
  }

  /** Run motor at a given percent output */
  public void run(double speed) {
    testMotor.set(speed); // speed between -1.0 and 1.0
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
