// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.vision.VisionConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.SimulationConstants;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.elevator.ArmSim;
import frc.robot.subsystems.elevator.ArmSim.ArmState;
import frc.robot.subsystems.elevator.ProjectileIntakeSim;
import frc.robot.subsystems.elevator.SSElevatorSim;
import frc.robot.subsystems.elevator.SSElevatorSim.ElevState;
import frc.robot.subsystems.vision.*;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // Subsystems
    private final Vision vision;
    private final Drive drive;
    private final ArmSim arm = ArmSim.getInstance();
    private final SSElevatorSim elevSim = SSElevatorSim.getInstance();
    private SwerveDriveSimulation driveSimulation = null;

    // Controller
    public final CommandXboxController controller = new CommandXboxController(0);
    public final CommandXboxController opController = new CommandXboxController(1);

    // Dashboard inputs
    private final LoggedDashboardChooser<Command> autoChooser;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        switch (Constants.currentMode) {
            case REAL:
                // Real robot, instantiate hardware IO implementations
                drive = new Drive(
                        new GyroIOPigeon2(),
                        new ModuleIOTalonFX(TunerConstants.FrontLeft),
                        new ModuleIOTalonFX(TunerConstants.FrontRight),
                        new ModuleIOTalonFX(TunerConstants.BackLeft),
                        new ModuleIOTalonFX(TunerConstants.BackRight),
                        (robotPose) -> {});
                vision = new Vision(
                        drive,
                        new VisionIOLimelight(VisionConstants.camera0Name, drive::getRotation),
                        new VisionIOQuestNav());
                break;

            case SIM:
                // Sim robot, instantiate physics sim IO implementations
                driveSimulation = new SwerveDriveSimulation(Drive.mapleSimConfig, new Pose2d(3, 3, new Rotation2d()));
                SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
                drive = new Drive(
                        new GyroIOSim(driveSimulation.getGyroSimulation()),
                        new ModuleIOSim(driveSimulation.getModules()[0]),
                        new ModuleIOSim(driveSimulation.getModules()[1]),
                        new ModuleIOSim(driveSimulation.getModules()[2]),
                        new ModuleIOSim(driveSimulation.getModules()[3]),
                        driveSimulation::setSimulationWorldPose);

                // photonvision sim is resource intensive
                // uncomment when needed
                vision = new Vision(
                        drive // ,
                        // new VisionIOPhotonVisionSim(
                        //         camera0Name, robotToCamera0, driveSimulation::getSimulatedDriveTrainPose),
                        // new VisionIOPhotonVisionSim(
                        //         camera1Name, robotToCamera1, driveSimulation::getSimulatedDriveTrainPose));
                        );

                ProjectileIntakeSim.createInstance(
                        driveSimulation::getSimulatedDriveTrainPose,
                        driveSimulation::getDriveTrainSimulatedChassisSpeedsFieldRelative);
                break;

            default:
                // Replayed robot, disable IO implementations
                drive = new Drive(
                        new GyroIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        (robotPose) -> {});
                vision = new Vision(drive, new VisionIO() {}, new VisionIO() {});

                break;
        }

        // Set up auto routines
        autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

        // Set up SysId routines
        autoChooser.addOption("Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
        autoChooser.addOption("Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
        autoChooser.addOption(
                "Drive SysId (Quasistatic Forward)", drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
                "Drive SysId (Quasistatic Reverse)", drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        autoChooser.addOption("Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption("Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by instantiating a
     * {@link GenericHID} or one of its subclasses ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}),
     * and then passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        // Default command, normal field-relative drive
        drive.setDefaultCommand(DriveCommands.joystickDrive(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> -controller.getRightX(),
                true));

        // robot oriented
        controller
                .leftBumper()
                .and(controller.a().negate())
                .whileTrue(DriveCommands.joystickDrive(
                        drive,
                        () -> -controller.getLeftY(),
                        () -> -controller.getLeftX(),
                        () -> -controller.getRightX(),
                        false));

        // aligning and field relative
        controller
                .a()
                .and(controller.leftBumper().negate())
                .whileTrue(DriveCommands.joystickDriveAtAngle(
                        drive,
                        () -> -controller.getLeftY(),
                        () -> -controller.getLeftX(),
                        () -> drive.getAlignAngle(),
                        true));

        // aligning and robot relative
        controller
                .a()
                .and(controller.leftBumper())
                .whileTrue(DriveCommands.joystickDriveAtAngle(
                        drive,
                        () -> -controller.getLeftY(),
                        () -> -controller.getLeftX(),
                        () -> drive.getAlignAngle(),
                        false));

        // Switch to X pattern when X button is pressed
        controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

        controller.b().whileTrue(Commands.run(() -> arm.armUp(), arm));

        controller.y().onTrue(Commands.runOnce(() -> arm.shootCoral(driveSimulation), arm));

        if (Constants.currentMode == Constants.Mode.SIM) {
            controller.leftBumper().onTrue(Commands.runOnce(() -> ProjectileIntakeSim.getInstance()
                    .dropCoralFromStation(false)));
            controller.rightBumper().onTrue(Commands.runOnce(() -> ProjectileIntakeSim.getInstance()
                    .dropCoralFromStation(true)));
            //     controller
            //             .rightBumper()
            //             .onTrue(Commands.runOnce(
            //                     () -> elevSim.setGoal(((SimulationConstants.MAX_HEIGHT - SimulationConstants.MIN_HEIGHT)
            //                                     * controller.getLeftTriggerAxis())
            //                             + SimulationConstants.MIN_HEIGHT)));
            opController
                    .y()
                    .onTrue(Commands.runOnce(() -> elevSim.setGoal(ElevState.L4))
                            .andThen(() -> arm.setSetpoint(ArmState.L4)));
            opController
                    .b()
                    .onTrue(Commands.runOnce(() -> elevSim.setGoal(ElevState.L3))
                            .andThen(() -> arm.setSetpoint(ArmState.L3)));
            opController
                    .a()
                    .onTrue(Commands.runOnce(() -> elevSim.setGoal(ElevState.L2))
                            .andThen(() -> arm.setSetpoint(ArmState.L2)));
            opController
                    .x()
                    .onTrue(Commands.runOnce(() -> elevSim.setGoal(ElevState.CoralStation))
                            .andThen(() -> arm.setSetpoint(ArmState.CoralStation)));
        }

        // Reset gyro / odometry
        final Runnable resetOdometry = Constants.currentMode == Constants.Mode.SIM
                ? () -> drive.resetOdometry(driveSimulation.getSimulatedDriveTrainPose())
                : () -> drive.resetOdometry(new Pose2d(drive.getPose().getTranslation(), new Rotation2d()));
        controller.start().onTrue(Commands.runOnce(resetOdometry).ignoringDisable(true));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.get();
    }

    public void resetSimulation() {
        if (Constants.currentMode != Constants.Mode.SIM) return;

        drive.resetOdometry(new Pose2d(3, 3, new Rotation2d()));
        SimulatedArena.getInstance().resetFieldForAuto();
    }

    public void displaySimFieldToAdvantageScope() {
        if (Constants.currentMode != Constants.Mode.SIM) return;

        Logger.recordOutput("FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
        Logger.recordOutput(
                "FieldSimulation/Coral", SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
        Logger.recordOutput(
                "FieldSimulation/Algae", SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
    }

    public static boolean isRedAlliance() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
    }
}
