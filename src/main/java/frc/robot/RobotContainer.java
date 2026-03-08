package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.drive.Drive;

public class RobotContainer {
  // Subsystems
  private final Drive drive = new Drive();
  private final Limelight limelight = new Limelight();
  private final Shooter shooter =
      new Shooter(
          Constants.FEED_MOTOR_ID,
          Constants.FEED_MOVE_MOTOR_ID,
          Constants.TRANSFER_MOTOR_ID,
          Constants.LAUNCH_MOTOR_ID);

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Auto chooser
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  public RobotContainer() {
    // Set up auto chooser
    Command simpleAuto =
        Commands.sequence(
            Commands.runEnd(
                    () -> drive.runVelocity(new ChassisSpeeds(1.5, 0.0, 0.39)),
                    () -> drive.runVelocity(new ChassisSpeeds(0.0, 0.0, 0.0)),
                    drive)
                .withTimeout(2.0),
            Commands.runEnd(() -> shooter.runShooter(0.6), shooter::stop, shooter)
                .withTimeout(1.0));

    autoChooser.setDefaultOption("Simple Shoot & Move", simpleAuto);
    SmartDashboard.putData("Auto Choices", autoChooser);

    configureButtonBindings();
  }

  private void configureButtonBindings() {
    // Default command: field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    // Lock to 0 degrees when A button is held
    controller
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> Rotation2d.kZero));

    // Left Bumper: FeedMove Forward
    controller
        .leftBumper()
        .whileTrue(
            Commands.runEnd(
                () -> {
                  if (!shooter.isFeedLimitFrontPressed()) {
                    shooter.runFeedMove(-0.55);
                  } else {
                    shooter.runFeedMove(0);
                  }
                },
                () -> shooter.runFeedMove(0),
                shooter));

    // Default shooter command: triggers + X button
    shooter.setDefaultCommand(
        Commands.run(
            () -> {
              double leftTrigger = controller.getLeftTriggerAxis();
              double rightTrigger = controller.getRightTriggerAxis();
              boolean isXPressed = controller.x().getAsBoolean();

              if (leftTrigger > 0.1) {
                shooter.runFeed(0.75);
              } else {
                shooter.runFeed(0);
              }

              if (rightTrigger > 0.1) {
                shooter.runShooter(0.85);
              } else {
                shooter.runShooter(0);
              }

              if (isXPressed) {
                shooter.runTransfer(-0.55);
              } else {
                shooter.runTransfer(0);
              }
            },
            shooter));

    // Right Bumper: FeedMove Backward
    controller
        .rightBumper()
        .whileTrue(
            Commands.runEnd(
                () -> {
                  if (!shooter.isFeedLimitBackPressed()) {
                    shooter.runFeedMove(0.75);
                  } else {
                    shooter.runFeedMove(0);
                  }
                },
                () -> shooter.runFeedMove(0),
                shooter));

    // B button: Run feed backwards
    controller
        .b()
        .whileTrue(
            Commands.runEnd(() -> shooter.runFeed(-0.55), () -> shooter.runFeed(0), shooter));

    // Right Trigger: Run shooter
    controller
        .rightTrigger()
        .whileTrue(Commands.runEnd(() -> shooter.runShooter(0.6), shooter::stop, shooter));

    // Y button: Align to AprilTag
    controller
        .y()
        .whileTrue(
            DriveCommands.alignToTarget(
                drive, limelight, () -> -controller.getLeftY(), () -> -controller.getLeftX()));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
