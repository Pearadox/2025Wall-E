// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drivers.PearadoxTalonFX;
import frc.robot.Constants.SimulationConstants;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;

public class ArmSim extends SubsystemBase {

    private DCMotor armGearbox = DCMotor.getKrakenX60(1);

    private SingleJointedArmSim armSim = new SingleJointedArmSim(
            armGearbox,
            SimulationConstants.ARM_GEARING,
            SingleJointedArmSim.estimateMOI(SimulationConstants.ARM_LENGTH, SimulationConstants.ARM_MASS),
            SimulationConstants.ARM_LENGTH,
            SimulationConstants.MIN_ANGLE,
            SimulationConstants.MAX_ANGLE,
            SimulationConstants.SIMULATE_GRAVITY,
            SimulationConstants.STARTING_ANGLE);

    private PearadoxTalonFX pivot;
    private TalonFXSimState pivotSim;

    private Mechanism2d mech2d = new Mechanism2d(6, 6);
    private MechanismRoot2d armPivot = mech2d.getRoot("Arm Pivot", 3, Units.inchesToMeters(74));
    private MechanismLigament2d elevator =
            armPivot.append(new MechanismLigament2d("Elevator", Units.inchesToMeters(74), -90));
    private MechanismLigament2d arm = armPivot.append(
            new MechanismLigament2d("Arm", SimulationConstants.ARM_LENGTH, 0, 3, new Color8Bit(Color.kYellow)));

    public static final ArmSim ARMSIM = new ArmSim();

    public static ArmSim getInstance() {
        return ARMSIM;
    }

    /** Creates a new ElevatorIOSim. */
    public ArmSim() {
        SmartDashboard.putData("Arm Sim", mech2d);
        pivot = new PearadoxTalonFX(0, NeutralModeValue.Brake, 80, true);
        pivotSim = new TalonFXSimState(pivot);

        Slot0Configs slot0Configs = new Slot0Configs();
        slot0Configs.kP = 1;
        slot0Configs.kG = 0;
        slot0Configs.kI = 0.1;
        slot0Configs.kD = 0;
        pivot.getConfigurator().apply(slot0Configs);
    }

    public void simulationPeriodic() {
        // This method will be called once per scheduler run

        SmartDashboard.putNumber("Position", pivot.getPosition().getValueAsDouble());

        armSim.setInput(pivotSim.getMotorVoltage());
        SmartDashboard.putNumber("Voltage", pivotSim.getMotorVoltage());

        armSim.update(0.02);

        SmartDashboard.putNumber("Angle", armSim.getAngleRads());
        SmartDashboard.putNumber("Angle Velocity", armSim.getVelocityRadPerSec());

        pivotSim.setSupplyVoltage(12);

        pivotSim.setRawRotorPosition(
                Units.radiansToRotations(armSim.getAngleRads() - 0) * SimulationConstants.ARM_GEARING);
        pivotSim.setRotorVelocity(
                Units.radiansToRotations(armSim.getVelocityRadPerSec() * SimulationConstants.ARM_GEARING));

        arm.setAngle(Units.radiansToDegrees(armSim.getAngleRads()));
    }

    public void reachSetpoint() {
        // var pidOutput = armController.calculate(encoder.getDistance(), Units.degreesToRadians(75));
    }

    public void armUp() {
        // pivot.set(0.5);
        final PositionVoltage request = new PositionVoltage(0).withSlot(0);
        pivot.setControl(request);
    }

    public void shootCoral(SwerveDriveSimulation driveSimulation) {
        SimulatedArena.getInstance()
                .addGamePieceProjectile(new ReefscapeCoralOnFly(
                        // Obtain robot position from drive simulation
                        driveSimulation.getSimulatedDriveTrainPose().getTranslation(),
                        // The scoring mechanism is installed at (0.46, 0) (meters) on the robot
                        new Translation2d(0.35, 0),
                        // Obtain robot speed from drive simulation
                        driveSimulation.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                        // Obtain robot facing from drive simulation
                        driveSimulation.getSimulatedDriveTrainPose().getRotation(),
                        // The height at which the coral is ejected
                        Meters.of(1.28),
                        // The initial speed of the coral
                        MetersPerSecond.of(2),
                        // The coral is ejected at a 35-degree slope
                        Degrees.of(-35)));
    }
}
