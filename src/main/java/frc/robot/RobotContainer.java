// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotContainer {
  // Subsystems
  private final Drive drive;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(DriveCommands.joystickDrive(
            drive,
            () -> applyExponent(-controller.getLeftY(), 2),
            () -> applyExponent(-controller.getLeftX(), 2),
            () -> applyExponent(-controller.getRightX(), 2),
            true));

    // left bumper = robot oriented, strafes
    controller
            .leftBumper()
            .and(controller.a().negate())
            .whileTrue(DriveCommands.joystickDrive(
                    drive,
                    () -> applyExponent(-controller.getLeftY(), 2),
                    () -> applyExponent(-controller.getLeftX(), 2),
                    () -> applyExponent(-controller.getRightX(), 2),
                    false));

    // a-button aligns to reef, overriding rotation
    controller
            .a()
            .and(controller.leftBumper().negate())
            .whileTrue(DriveCommands.joystickDriveAtAngle(
                    drive,
                    () -> applyExponent(-controller.getLeftY(), 2, 0.2),
                    () -> applyExponent(-controller.getLeftX(), 2, 0.2),
                    () -> drive.getAlignAngleReef(),
                    true));

    controller
            .y()
            .and(controller.leftBumper().negate())
            .whileTrue(DriveCommands.joystickDriveAtAngle(
                    drive,
                    // () -> applyExponent(-controller.getLeftY(), 2),
                    () -> applyExponent(drive.getAlignForwardSpeedPercent(), 1, 0.1),
                    // () -> applyExponent(-controller.getLeftX(), 2),
                    () -> applyExponent(drive.getAlignStrafeSpeedPercent(), 1, 0.1),
                    () -> drive.getAlignAngleReef(),
                    false));

    // aligns while robot-oriented when both left bumper and a pressed
    controller
            .a()
            .and(controller.leftBumper())
            .whileTrue(DriveCommands.joystickDriveAtAngle(
                    drive,
                    () -> applyExponent(-controller.getLeftY(), 2, 0.15),
                    () -> applyExponent(-controller.getLeftX(), 2, 0.15),
                    () -> drive.getAlignAngleReef(),
                    false));


    controller
            .leftTrigger()
            .whileTrue(DriveCommands.joystickDrive(
                drive, 
                () -> 0, 
                () -> -controller.getLeftTriggerAxis() + DriveConstants.ROBOT_ORIENTED_TRIGGER_OFFSET, 
                () -> -controller.getRightX(), 
                false));

    controller
            .rightTrigger()
            .whileTrue(DriveCommands.joystickDrive(
                drive, 
                () -> 0, 
                () -> controller.getRightTriggerAxis() - DriveConstants.ROBOT_ORIENTED_TRIGGER_OFFSET, 
                () -> -controller.getRightX(), 
                false));

    // // y-button aligns to coral station, overriding rotation
    // controller
    //         .a()
    //         .and(controller.y())
    //         .whileTrue(DriveCommands.joystickDriveAtAngle(
    //                 drive,
    //                 () -> applyExponent(-controller.getLeftY(), 2, 0.15),
    //                 () -> applyExponent(-controller.getLeftX(), 2, 0.15),
    //                 () -> drive.getAlignAngleCoralStation(),
    //                 false));

    // Switch to X pattern when X button is pressed
    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when B button is pressed
    controller
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new PathPlannerAuto("Test Auto");
    //return autoChooser.get();
  }
  
  public static boolean isRedAlliance() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
    }

    public static double applyExponent(double percent, double exponent) {
        return applyExponent(percent, exponent, 0.07);
    }

    public static double applyExponent(double percent, double exponent, double deadband) {
        return Math.abs(percent) > deadband ? Math.copySign(Math.pow(percent, exponent), percent) : 0;
    }
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.03).withRotationalDeadband(MaxAngularRate * 0.03) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-Math.pow(joystick.getLeftY(), 2.0) * Math.signum(joystick.getLeftY()) * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-Math.pow(joystick.getLeftX(), 2.0) * Math.signum(joystick.getLeftX()) * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
