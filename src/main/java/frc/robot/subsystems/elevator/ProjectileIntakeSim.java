package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.SimulationConstants;
import java.util.HashSet;
import java.util.Set;
import java.util.function.Supplier;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.gamepieces.GamePieceProjectile;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;
import org.littletonrobotics.junction.Logger;

public class ProjectileIntakeSim {
    private static ProjectileIntakeSim instance;

    public static ProjectileIntakeSim createInstance(
            Supplier<Pose2d> poseSupplier, Supplier<ChassisSpeeds> chassisSpeedsSupplier) {
        if (instance == null) {
            instance = new ProjectileIntakeSim(poseSupplier, chassisSpeedsSupplier);
        } else {
            throw new IllegalStateException("ProjectileIntakeSim instance already created!");
        }
        return instance;
    }

    public static ProjectileIntakeSim getInstance() {
        if (instance == null) {
            throw new IllegalStateException("ProjectileIntakeSim instance has not been created!");
        }
        return instance;
    }

    final Transform3d INTAKE_TRANSFORM =
            new Transform3d(new Translation3d(-0.3, 0, Units.inchesToMeters(12)), new Rotation3d(0, 90, 0));

    // must be within 6 inches of the intake
    final double TRANSLATIONAL_TOLERANCE_METERS = Units.inchesToMeters(6);
    // any rotation is fine
    final double ROTATIONAL_TOLERANCE_RADIANS = Units.degreesToRadians(10000);

    private boolean hasCoral = false; // max 1 coral
    private double elevHeight = SimulationConstants.STARTING_HEIGHT;
    private double armAngle = SimulationConstants.STARTING_ANGLE;
    private double camAngle = Units.degreesToRadians(0);

    private Timer intakingTimer = new Timer();
    private final double intakingTime = 0.75;

    // cooldown between shooting and rerunning the intake
    // TODO: ee wheel sim
    private Timer shootingTimer = new Timer();
    private final double shootingTime = 0.75;
    private boolean disableIntake = false;

    private Supplier<Pose2d> poseSupplier;
    private Supplier<ChassisSpeeds> chassisSpeedsSupplier;

    private ProjectileIntakeSim(Supplier<Pose2d> poseSupplier, Supplier<ChassisSpeeds> chassisSpeedsSupplier) {
        this.poseSupplier = poseSupplier;
        this.chassisSpeedsSupplier = chassisSpeedsSupplier;
    }

    public void periodic() {
        visualizeHeldCoral();

        // cooldown between shooting and restarting the intake
        // so you don't immediately reintake what you
        if (shootingTimer.get() > shootingTime) {
            shootingTimer.stop();
            shootingTimer.reset();
            disableIntake = false;
        } else if (shootingTimer.get() > 0) {
            disableIntake = true;
        }

        if (disableIntake || hasCoral) return; // max 1 coral

        Pose3d intakePose = getIntakePose();
        Set<GamePieceProjectile> gamePieceProjectiles =
                SimulatedArena.getInstance().gamePieceLaunched();
        Set<GamePieceProjectile> toRemove = new HashSet<>();

        for (GamePieceProjectile gamePiece : gamePieceProjectiles) {
            if (gamePiece instanceof ReefscapeCoralOnFly) {
                if (checkCoralPlacement(intakePose.minus(gamePiece.getPose3d()))) {
                    toRemove.add(gamePiece);
                    break; // max 1 coral
                }
            }
        }

        for (GamePieceProjectile gamePiece : toRemove) {
            gamePieceProjectiles.remove(gamePiece);
            hasCoral = true;
            intakingTimer.restart();
            break; // max 1 coral
        }
    }

    public boolean checkCoralPlacement(Transform3d difference) {
        boolean poseWithinTolerance = difference.getTranslation().getNorm() < TRANSLATIONAL_TOLERANCE_METERS
                && Math.abs(difference.getRotation().getX()) < ROTATIONAL_TOLERANCE_RADIANS
                && Math.abs(difference.getRotation().getY()) < ROTATIONAL_TOLERANCE_RADIANS
                && Math.abs(difference.getRotation().getZ()) < ROTATIONAL_TOLERANCE_RADIANS;
        return poseWithinTolerance;
    }

    public void updateElevatorHeight(double elevatorHeight) {
        this.elevHeight = elevatorHeight;
    }

    public void updateArmAngle(double armAngleRads) {
        this.armAngle = armAngleRads;
    }

    public void updateCamAngle(double camAngleRads) {
        this.camAngle = camAngleRads;
    }

    public Transform3d getEndEffectorTransform() {
        // return new Transform3d(
        //         new Translation3d(PIVOT_TO_EE, 0, 0) // EE is 12in from pivot
        //                 .rotateBy(new Rotation3d(0, -armAngle, 0)) // rotate around pivot
        //                 .plus(new Translation3d(0, 0, elevHeight)), // robot to pivot translation
        //         new Rotation3d(0, Units.degreesToRadians(90 - 35), 0)); // constant rotation

        Translation3d armPivot = new Translation3d(0, 0, elevHeight);

        Translation3d armEnd =
                new Translation3d(SimulationConstants.ARM_LENGTH, 0, 0).rotateBy(new Rotation3d(0, -armAngle, 0));

        Rotation3d camRot = new Rotation3d(0, -camAngle, 0);

        Translation3d coralToArmEnd = new Translation3d(
                        SimulationConstants.EE_TO_CORAL_HEIGHT + SimulationConstants.CORAL_LENGTH / 2.0,
                        0,
                        SimulationConstants.EE_TO_CORAL_WIDTH)
                .rotateBy(camRot);

        return new Transform3d(armPivot.plus(armEnd).plus(coralToArmEnd), camRot);
    }

    public Pose3d getEndEffectorPose() {
        Pose3d robotPose = new Pose3d(poseSupplier.get());
        return robotPose.transformBy(getEndEffectorTransform());
    }

    public Pose3d getIntakePose() {
        // Pose3d robotPose = new Pose3d(poseSupplier.get());
        // return robotPose.transformBy(INTAKE_TRANSFORM);
        return getEndEffectorPose();
    }

    public void visualizeHeldCoral() {
        // TODO: coral to intake pose
        if (intakingTimer.isRunning()) {
            if (intakingTimer.get() > intakingTime) {
                intakingTimer.stop();
                intakingTimer.reset();
            }
            Logger.recordOutput(
                    "FieldSimulation/Held Coral",
                    getIntakePose().interpolate(getEndEffectorPose(), intakingTimer.get() / intakingTime));
        } else if (hasCoral) {
            Logger.recordOutput("FieldSimulation/Held Coral", getEndEffectorPose());
        } else {
            Logger.recordOutput("FieldSimulation/Held Coral", Pose3d.kZero);
        }
    }

    public void shootCoral() {
        // if (!hasCoral) return;
        Transform3d eeTransform = getEndEffectorTransform();

        SimulatedArena.getInstance()
                .addGamePieceProjectile(new ReefscapeCoralOnFly(
                        // Obtain robot position from drive simulation
                        poseSupplier.get().getTranslation(),
                        // The scoring mechanism is installed at this position on the robot
                        eeTransform.getTranslation().toTranslation2d(),
                        // Obtain robot speed from drive simulation
                        chassisSpeedsSupplier.get(),
                        // Obtain robot facing from drive simulation
                        poseSupplier.get().getRotation(),
                        // The height at which the coral is ejected
                        eeTransform.getMeasureZ(),
                        // The initial speed of the coral
                        MetersPerSecond.of(2),
                        // The coral is ejected at this angle
                        eeTransform.getRotation().getMeasureAngle().unaryMinus()));

        hasCoral = false;
        disableIntake = true;
        shootingTimer.restart();
    }

    public void dropCoralFromStation(boolean isProcessorSide) {
        SimulatedArena.getInstance()
                .addGamePieceProjectile(new ReefscapeCoralOnFly(
                        // Obtain robot position from drive simulation
                        isProcessorSide ? FieldConstants.BLUE_PS_CORAL_STATION : FieldConstants.BLUE_NPS_CORAL_STATION,
                        // The scoring mechanism is installed at this position on the robot
                        Translation2d.kZero,
                        // Obtain robot speed from drive simulation
                        new ChassisSpeeds(),
                        // Obtain robot facing from drive simulation
                        Rotation2d.fromDegrees(isProcessorSide ? -35 : 35),
                        // The height at which the coral is ejected
                        Meters.of(1),
                        // The initial speed of the coral
                        MetersPerSecond.of(0),
                        // The coral is ejected at this angle
                        Degrees.of(0)));
    }
}
