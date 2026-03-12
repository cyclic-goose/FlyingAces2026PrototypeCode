package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.drive.Drive;

public class RobotContainer {
  // Subsystems
  private final Drive drive = new Drive();
  private final Limelight limelight = new Limelight();

  // now separate intake and shooter
  private final Intake intake = new Intake(Constants.FEED_MOTOR1_ID, Constants.FEED_MOTOR2_ID, Constants.FEED_MOVE_MOTOR_ID);
  private final Shooter shooter =
      new Shooter(Constants.TRANSFER_MOTOR_ID, Constants.LAUNCH_MOTOR_ID);

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

    // Auto: back up 1m, align to goal via limelight, rev then fire
    Command alignAndShootAuto =
        Commands.sequence(
                // 1. Drive backwards ~1 meter (negative X = backwards)
                Commands.runEnd(
                        () -> drive.runVelocity(new ChassisSpeeds(-1.5, 0.0, 0.0)),
                        () -> drive.runVelocity(new ChassisSpeeds(0.0, 0.0, 0.0)),
                        drive)
                    .withTimeout(0.67),

                // 2. Center on closest visible tag
                DriveCommands.centerOnTag(drive, limelight).withTimeout(5.0),

                // 3. Rev launch motor for 1 second (let it spin up)
                Commands.runEnd(() -> shooter.runShooter(0.85), () -> {}, shooter).withTimeout(1.0),

                // 4. Fire: run launch + transfer together, feed ball into shooter
                Commands.parallel(
                        Commands.runEnd(() -> shooter.runShooter(0.85), shooter::stop, shooter),
                        Commands.runEnd(() -> intake.runFeed(0.75), intake::stop, intake))
                    .withTimeout(3.0))
            .withTimeout(15.0); // leave margin within 20s auto period

    autoChooser.setDefaultOption("Simple Shoot & Move", simpleAuto);
    autoChooser.addOption("Align & Shoot", alignAndShootAuto);
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
                  if (!intake.isFeedLimitFrontPressed()) {
                    intake.runFeedMove(-0.55);
                  } else {
                    intake.runFeedMove(0);
                  }
                },
                () -> intake.runFeedMove(0),
                intake));

    // Right Bumper: FeedMove Backward
    controller
        .rightBumper()
        .whileTrue(
            Commands.runEnd(
                () -> {
                  if (!intake.isFeedLimitBackPressed()) {
                    intake.runFeedMove(0.75);
                  } else {
                    intake.runFeedMove(0);
                  }
                },
                () -> intake.runFeedMove(0),
                intake));

    // B button: Run feed backwards
    controller
        .b()
        .whileTrue(Commands.runEnd(() -> intake.runFeed(-0.55), () -> intake.runFeed(0), intake));

    // Default intake command: left trigger runs feed motor
    intake.setDefaultCommand(
        Commands.run(
            () -> {
              double rightTrigger = controller.getRightTriggerAxis();
              if (rightTrigger > 0.1 && !intake.isFeedLimitBackPressed()) {
                intake.runFeed(0.55);
              } else {
                intake.runFeed(0);
              }
            },
            intake));

    // Default shooter command: right trigger runs launch, X runs transfer
    /*shooter.setDefaultCommand(
    Commands.run(
        () -> {
          double rightTrigger = controller.getRightTriggerAxis();
          boolean isXPressed = controller.x().getAsBoolean();

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
        shooter));*/

    // Y button: Center on closest visible AprilTag (with joystick driving)
    controller
        .y()
        .whileTrue(
            DriveCommands.centerOnTag(
                drive, limelight, () -> -controller.getLeftY(), () -> -controller.getLeftX()));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
