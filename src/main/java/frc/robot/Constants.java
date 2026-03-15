package frc.robot;

/** Robot-wide constants. */
public final class Constants {
  // Shooter motor CAN IDs
  public static final int FEED_MOTOR1_ID = 15;
  public static final int FEED_MOTOR2_ID = 16;
  public static final int FEED_MOVE_MOTOR_ID = 10;
  public static final int TRANSFER_MOTOR_ID = 17;
  public static final int LAUNCH_MOTOR_ID = 18;

  public static final double autoRotateMagnitudeRadians = 3.0 * 0.78;
  public static final double autoForwardSpeed = 3.9; // "m/s"
  public static final double autoSpeedAdjust = 0.15;
  public static final double autoForwardTime = 3.4; // total seconds

  // for feeding section of auto
  public static final double autoFeedingForwardSpeed = 2.0; // in meters per second
  public static final double autoFeedingForwardTime = 2.0; // in seconds

  public static boolean lockIntake = false;
}
