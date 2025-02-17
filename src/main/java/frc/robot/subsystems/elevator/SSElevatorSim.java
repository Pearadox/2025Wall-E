// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drivers.PearadoxTalonFX;
import frc.robot.Constants.SimulationConstants;
import org.littletonrobotics.junction.Logger;

public class SSElevatorSim extends SubsystemBase {
    private DCMotor elevatorGearbox = DCMotor.getKrakenX60(2);

    private final ElevatorSim elevatorSim = new ElevatorSim(
            elevatorGearbox,
            SimulationConstants.ELEVATOR_GEARING,
            SimulationConstants.CARRIAGE_MASS,
            SimulationConstants.DRUM_RADIUS,
            SimulationConstants.MIN_HEIGHT,
            SimulationConstants.MAX_HEIGHT,
            SimulationConstants.SIMULATE_GRAVITY,
            SimulationConstants.STARTING_HEIGHT);

    private PearadoxTalonFX elevatorMotor;
    private TalonFXSimState elevatorMotorSimState;

    private double goal = SimulationConstants.MIN_HEIGHT;

    public enum ElevState {
        L4(72),
        L3(64.87),
        L2(49),
        CoralStation(45),
        Stowed(42);

        public final double height;

        private ElevState(double heightInches) {
            this.height = Units.inchesToMeters(heightInches);
        }
    }

    public static final SSElevatorSim SS_ELEVATOR_SIM = new SSElevatorSim();

    public static SSElevatorSim getInstance() {
        return SS_ELEVATOR_SIM;
    }

    /** Creates a new Elevator. */
    private SSElevatorSim() {
        elevatorMotor = new PearadoxTalonFX(1, NeutralModeValue.Brake, 80, false);
        elevatorMotorSimState = elevatorMotor.getSimState();

        Slot0Configs slot0Configs = new Slot0Configs();
        // slot0Configs.kG = 0.75;
        // slot0Configs.kV = 3;
        slot0Configs.kP = 25;
        slot0Configs.kI = 0.1;
        slot0Configs.kD = 0;
        elevatorMotor.getConfigurator().apply(slot0Configs);
    }

    public void simulationPeriodic() {
        // This method will be called once per scheduler run
        elevatorMotorSimState.setSupplyVoltage(12);

        reachGoal();

        double volts = elevatorMotorSimState.getMotorVoltage() * 6;
        elevatorSim.setInput(volts);
        Logger.recordOutput("Elevator/Volts", volts);

        elevatorSim.update(0.02);

        elevatorMotorSimState.setRawRotorPosition(getMotorRotations(elevatorSim.getPositionMeters()));

        // angular velocity = linear velocity / radius
        elevatorMotorSimState.setRotorVelocity(
                (elevatorSim.getVelocityMetersPerSecond() / SimulationConstants.DRUM_RADIUS)
                        // radians/sec to rotations/sec
                        / (2 * Math.PI)
                        / SimulationConstants.ARM_GEARING);

        MechVisualizer.getInstance().updateElevatorHeight(elevatorSim.getPositionMeters());
    }

    public void reachGoal() {
        // double volts = pidController.calculate(elevatorSim.getPositionMeters(), goal);
        // volts = MathUtil.clamp(volts, -12, 12);
        // elevatorSim.setInputVoltage(volts);
        final PositionVoltage request = new PositionVoltage(getMotorRotations(goal)).withSlot(0);
        elevatorMotor.setControl(request);

        Logger.recordOutput("Elevator/Goal", goal);
        Logger.recordOutput("Elevator/Pos", elevatorSim.getPositionMeters());
    }

    public void setGoal(double goal) {
        this.goal = goal;
    }

    public void setGoal(ElevState elevState) {
        this.goal = elevState.height;
    }

    public void stop() {
        elevatorSim.setInputVoltage(0);
    }

    public static double getMotorRotations(double linearDisplacement) {
        // angular displacement in radians = linear displacement / radius
        return Units.radiansToRotations(linearDisplacement / SimulationConstants.DRUM_RADIUS)
                // divide by gear ratio
                / SimulationConstants.ELEVATOR_GEARING;
    }
}
