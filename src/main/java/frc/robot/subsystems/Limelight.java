package frc.robot.subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.Logger;

public class Limelight extends SubsystemBase {
  private final NetworkTable table;
  private final Drive drive;

  // Standard deviation scaling constants
  // Base trust at 1 meter distance; scales linearly with distance squared
  private static final double XY_STD_DEV_BASE = 0.3;
  private static final double THETA_STD_DEV_BASE = 0.5;
  // Multi-tag readings are more reliable
  private static final double MULTI_TAG_SCALE = 0.5;

  public Limelight(Drive drive) {
    this.drive = drive;
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

  /** Returns the number of AprilTags seen in the current MegaTag solution. */
  public int getTagCount() {
    double[] data = table.getEntry("botpose_wpiblue").getDoubleArray(new double[0]);
    // Tag count is at index 7 in the botpose array
    if (data.length >= 8) {
      return (int) data[7];
    }
    return 0;
  }

  /**
   * Returns the robot's field-relative pose calculated by MegaTag. Returns null if no target.
   * Origin is always Blue Alliance Wall (standard wpiblue).
   */
  public Pose2d getBotPose() {
    double[] data = table.getEntry("botpose_wpiblue").getDoubleArray(new double[0]);

    if (data.length < 7) return null;

    // Reject if all zeros (no valid solution)
    if (data[0] == 0.0 && data[1] == 0.0 && data[5] == 0.0) return null;

    return new Pose2d(new Translation2d(data[0], data[1]), Rotation2d.fromDegrees(data[5]));
  }

  /**
   * Returns the MegaTag capture latency in seconds. The botpose array index 6 contains total
   * latency in milliseconds (capture + pipeline).
   */
  public double getLatencySeconds() {
    double[] data = table.getEntry("botpose_wpiblue").getDoubleArray(new double[0]);
    if (data.length >= 7) {
      return data[6] / 1000.0;
    }
    return 0.0;
  }

  /**
   * Computes vision measurement standard deviations scaled by distance to the target. Farther
   * targets and single-tag solutions get higher (less trusted) standard deviations.
   */
  private Matrix<N3, N1> getStdDevs(Pose2d visionPose) {
    int tagCount = getTagCount();
    if (tagCount == 0) {
      return VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    }

    // Compute average distance from robot to visible tags using the pose estimate
    // Use distance from center of field as a rough proxy (actual tag positions would be better)
    double avgDist = visionPose.getTranslation().getDistance(drive.getPose().getTranslation());

    // Scale standard deviations: trust decreases with distance squared
    double distScale = 1.0 + (avgDist * avgDist);
    double tagScale = tagCount >= 2 ? MULTI_TAG_SCALE : 1.0;

    double xyStdDev = XY_STD_DEV_BASE * distScale * tagScale;
    double thetaStdDev = THETA_STD_DEV_BASE * distScale * tagScale;

    return VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev);
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Limelight/tx", getTx());
    Logger.recordOutput("Limelight/ty", getTy());
    Logger.recordOutput("Limelight/hasTarget", hasTarget());

    if (!hasTarget()) return;

    Pose2d botPose = getBotPose();
    if (botPose == null) return;

    Logger.recordOutput("Limelight/BotPose", botPose);
    Logger.recordOutput("Limelight/TagCount", getTagCount());

    // Compute the timestamp when the image was actually captured
    double timestamp = Timer.getFPGATimestamp() - getLatencySeconds();

    // Feed the vision measurement into the drive pose estimator
    Matrix<N3, N1> stdDevs = getStdDevs(botPose);
    drive.addVisionMeasurement(botPose, timestamp, stdDevs);
  }
}
