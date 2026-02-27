package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Limelight extends SubsystemBase {
  private final NetworkTable table;

  public Limelight() {
    table = NetworkTableInstance.getDefault().getTable("limelight");
  }

  /** Horizontal offset from crosshair to target in degrees. */
  public double getTx() {
    return table.getEntry("tx").getDouble(0.0);
  }

  /** Vertical offset from crosshair to target in degrees. */
  public double getTy() {
    return table.getEntry("ty").getDouble(0.0);
  }

  /** Whether the Limelight sees a valid target. */
  public boolean hasTarget() {
    return table.getEntry("tv").getDouble(0.0) == 1.0;
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Limelight/tx", getTx());
    Logger.recordOutput("Limelight/ty", getTy());
    Logger.recordOutput("Limelight/hasTarget", hasTarget());
  }
}
