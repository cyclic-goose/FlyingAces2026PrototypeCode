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

  // Base standard deviation at 1 meter; scales with distance squared
  private static final double XY_STD_DEV_BASE = 0.3;

  // Multi-tag readings are more reliable
  private static final double MULTI_TAG_SCALE = 0.5;

  // Reject single-tag readings beyond this distance (meters)
  private static final double MAX_SINGLE_TAG_DISTANCE = 4.0;

  // Reject vision poses outside field boundaries (meters)
  // 2025 FRC field is ~16.54m x 8.02m, with some margin
  private static final double FIELD_X_MAX = 17.0;
  private static final double FIELD_Y_MAX = 8.5;

  // Camera mounting position relative to robot center (meters and degrees)
  // TODO: Measure these on your actual robot
  private static final double CAM_FORWARD = 0.3; // meters forward from robot center
  private static final double CAM_RIGHT = 0.0; // meters right from robot center
  private static final double CAM_UP = 0.5; // meters up from the floor
  private static final double CAM_ROLL = 0.0; // degrees
  private static final double CAM_PITCH = 15.0; // degrees up from horizontal
  private static final double CAM_YAW = 0.0; // degrees (0 = facing forward)

  public Limelight(Drive drive) {
    this.drive = drive;
    table = NetworkTableInstance.getDefault().getTable("limelight");

    table
        .getEntry("camerapose_robotspace_set")
        .setDoubleArray(
            new double[] {CAM_FORWARD, CAM_RIGHT, CAM_UP, CAM_ROLL, CAM_PITCH, CAM_YAW});

    table.getEntry("robot_orientation_set").setDoubleArray(new double[] {0, 0, 0, 0, 0, 0});
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

  /** Returns the primary target AprilTag ID, or -1 if none. */
  public double getTagID() {
    return table.getEntry("tid").getDouble(-1);
  }

  /** Returns the target area (0-100% of image). Larger = closer to target. */
  public double getTargetArea() {
    return table.getEntry("ta").getDouble(0.0);
  }

  /** Returns the number of AprilTags seen in the current MegaTag2 solution. */
  public int getTagCount() {
    double[] data = table.getEntry("botpose_orb_wpiblue").getDoubleArray(new double[0]);
    if (data.length >= 8) {
      return (int) data[7];
    }
    return 0;
  }

  /** Returns the average tag distance from the botpose data, or -1 if unavailable. */
  public double getAvgTagDistance() {
    double[] data = table.getEntry("botpose_orb_wpiblue").getDoubleArray(new double[0]);
    if (data.length >= 10) {
      return data[9]; // index 9 = average tag distance in meters
    }
    return -1;
  }

  /**
   * Returns the robot's field-relative pose from MegaTag2. Returns null if no valid solution. Uses
   * botpose_orb_wpiblue which relies on the gyro for rotation (fed via robot_orientation_set).
   */
  public Pose2d getBotPose() {
    double[] data = table.getEntry("botpose_orb_wpiblue").getDoubleArray(new double[0]);
    if (data.length < 7) return null;
    if (data[0] == 0.0 && data[1] == 0.0 && data[5] == 0.0) return null;
    return new Pose2d(new Translation2d(data[0], data[1]), Rotation2d.fromDegrees(data[5]));
  }

  /**
   * Returns the MegaTag2 capture latency in seconds. Index 6 contains total latency in
   * milliseconds.
   */
  public double getLatencySeconds() {
    double[] data = table.getEntry("botpose_orb_wpiblue").getDoubleArray(new double[0]);
    if (data.length >= 7) {
      return data[6] / 1000.0;
    }
    return 0.0;
  }

  /** Returns true if the vision pose is within the field boundaries. */
  private boolean isOnField(Pose2d pose) {
    double x = pose.getX();
    double y = pose.getY();
    return x >= 0 && x <= FIELD_X_MAX && y >= 0 && y <= FIELD_Y_MAX;
  }

  /**
   * Computes vision measurement standard deviations scaled by actual tag distance. MegaTag2 uses
   * the gyro for rotation, so theta std dev is MAX_VALUE to prevent vision from overriding heading.
   */
  private Matrix<N3, N1> getStdDevs() {
    int tagCount = getTagCount();
    if (tagCount == 0) {
      return VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    }

    double avgDist = getAvgTagDistance();
    if (avgDist < 0) avgDist = 4.0; // fallback if unavailable

    double distScale = 1.0 + (avgDist * avgDist);
    double tagScale = tagCount >= 2 ? MULTI_TAG_SCALE : 1.0;
    double xyStdDev = XY_STD_DEV_BASE * distScale * tagScale;

    return VecBuilder.fill(xyStdDev, xyStdDev, Double.MAX_VALUE);
  }

  @Override
  public void periodic() {
    // Feed current gyro heading to the Limelight for MegaTag2
    double yawDegrees = drive.getRotation().getDegrees();
    table
        .getEntry("robot_orientation_set")
        .setDoubleArray(new double[] {yawDegrees, 0, 0, 0, 0, 0});

    Logger.recordOutput("Limelight/tx", getTx());
    Logger.recordOutput("Limelight/ty", getTy());
    Logger.recordOutput("Limelight/hasTarget", hasTarget());

    if (!hasTarget()) return;

    int tagCount = getTagCount();
    double avgDist = getAvgTagDistance();

    Logger.recordOutput("Limelight/TagCount", tagCount);
    Logger.recordOutput("Limelight/AvgTagDistance", avgDist);

    // Reject single-tag readings that are too far away (unreliable with one camera)
    if (tagCount == 1 && avgDist > MAX_SINGLE_TAG_DISTANCE) {
      Logger.recordOutput("Limelight/Rejected", "single tag too far");
      return;
    }

    Pose2d botPose = getBotPose();
    if (botPose == null) return;

    // Reject poses that are clearly off-field
    if (!isOnField(botPose)) {
      Logger.recordOutput("Limelight/Rejected", "off field");
      return;
    }

    Logger.recordOutput("Limelight/BotPose", botPose);

    double timestamp = Timer.getFPGATimestamp() - getLatencySeconds();
    Matrix<N3, N1> stdDevs = getStdDevs();
    drive.addVisionMeasurement(botPose, timestamp, stdDevs);
  }
}
