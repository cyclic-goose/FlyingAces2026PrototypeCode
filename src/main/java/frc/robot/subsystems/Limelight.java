package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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

  /**
   * Returns the robot's field-relative pose calculated by MegaTag. Returns null if no target.
   * Origin is always Blue Alliance Wall (standard wpiblue).
   */
  public Pose2d getBotPose() {
    // [x, y, z, roll, pitch, yaw, latency, ...]
    double[] data = table.getEntry("botpose_wpiblue").getDoubleArray(new double[0]);

    if (data.length < 6) return null;

    // Extract X, Y, and Yaw (converted to Rotation2d)
    return new Pose2d(new Translation2d(data[0], data[1]), Rotation2d.fromDegrees(data[5]));
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Limelight/tx", getTx());
    Logger.recordOutput("Limelight/ty", getTy());
    Logger.recordOutput("Limelight/hasTarget", hasTarget());

    Pose2d botPose = getBotPose();
    if (botPose != null) {
      Logger.recordOutput("Limelight/BotPose", botPose);
    }
  }
}
