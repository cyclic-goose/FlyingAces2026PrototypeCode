package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Set;

public class Limelight extends SubsystemBase {
  private final NetworkTable table;

  private static final Set<Integer> GOAL_TAG_IDS =
      Set.of(2, 3, 4, 5, 8, 9, 10, 11, 18, 19, 20, 21, 24, 25, 26, 27);

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

  /** Returns the primary target AprilTag ID, or -1 if none. */
  public double getTagID() {
    return table.getEntry("tid").getDouble(-1);
  }

  /** Whether the Limelight sees a valid goal tag (not driver station tags). */
  public boolean hasGoalTarget() {
    return hasTarget() && GOAL_TAG_IDS.contains((int) getTagID());
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Limelight/tx", getTx());
    SmartDashboard.putNumber("Limelight/ty", getTy());
    SmartDashboard.putBoolean("Limelight/hasTarget", hasTarget());
    SmartDashboard.putNumber("Limelight/TagID", getTagID());
  }
}
