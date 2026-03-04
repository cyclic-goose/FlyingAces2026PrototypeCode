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

  // Multi-tag readings are more reliable
  private static final double MULTI_TAG_SCALE = 0.5;

  // Camera mounting position relative to robot center (meters and degrees)
  // Adjust these to match your physical Limelight placement
  private static final double CAM_FORWARD = 0.3; // meters forward from robot center
  private static final double CAM_RIGHT = 0.0; // meters right from robot center
  private static final double CAM_UP = 0.5; // meters up from the floor
  private static final double CAM_ROLL = 0.0; // degrees
  private static final double CAM_PITCH = 15.0; // degrees up from horizontal
  private static final double CAM_YAW = 0.0; // degrees (0 = facing forward)

  public Limelight(Drive drive) {
    this.drive = drive;
    table = NetworkTableInstance.getDefault().getTable("limelight");

    // Tell the Limelight where the camera is mounted on the robot
    table
        .getEntry("camerapose_robotspace_set")
        .setDoubleArray(
            new double[] {CAM_FORWARD, CAM_RIGHT, CAM_UP, CAM_ROLL, CAM_PITCH, CAM_YAW});

    // Use MegaTag2: supply the gyro heading so the Limelight uses it for rotation
    // instead of computing rotation from vision (which fights the gyro)
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

  public double getTagID() {
    return table.getEntry("tid").getDouble(-1);
  }

  @Override
  public void periodic() {
    // Feed current gyro heading to the Limelight for MegaTag2
    // Format: [yaw, yawRate, pitch, pitchRate, roll, rollRate]
    double yawDegrees = drive.getRotation().getDegrees();
    table
        .getEntry("robot_orientation_set")
        .setDoubleArray(new double[] {yawDegrees, 0, 0, 0, 0, 0});

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
