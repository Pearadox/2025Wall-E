// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

    // private final Mechanism2d mech2d = new Mechanism2d(Units.inchesToMeters(10), Units.inchesToMeters(90));
    // private final MechanismRoot2d elevatorRoot =
    //         mech2d.getRoot("Elevator Root", Units.inchesToMeters(5), Units.inchesToMeters(0.5));
    // private final MechanismLigament2d elevator2d =
    //         elevatorRoot.append(new MechanismLigament2d("Elevator", elevatorSim.getPositionMeters(), 90));

    // private PearadoxTalonFX elevator;
    // private TalonFXSimState elevatorSimState;

    private PIDController pidController = new PIDController(10, 0, 0);

    private double goal = SimulationConstants.MIN_HEIGHT;

    public enum ElevState {
        L4(72),
        L3(64.87),
        L2(49),
        CoralStation(45);

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
        // elevator = new PearadoxTalonFX(1, NeutralModeValue.Brake, 80, true);
        // elevatorSimState = elevator.getSimState();
        // SmartDashboard.putData("Elevator Sim", mech2d);
    }

    public void simulationPeriodic() {
        // This method will be called once per scheduler run
        // elevatorSim.setInput(elevatorSimState.getMotorVoltage());

        reachGoal();
        elevatorSim.update(0.02);

        // elevatorSimState.setSupplyVoltage(12);
        // elevatorSimState.setRawRotorPosition(null);
        // elevatorSimState.setRotorVelocity(null);

        // elevator2d.setLength(elevatorSim.getPositionMeters());
        // SmartDashboard.putData("Elevator Sim", mech2d);

        MechVisualizer.getInstance().updateElevatorHeight(elevatorSim.getPositionMeters());
    }

    public void reachGoal() {
        double volts = pidController.calculate(elevatorSim.getPositionMeters(), goal);
        volts = MathUtil.clamp(volts, -12, 12);
        elevatorSim.setInputVoltage(volts);

        Logger.recordOutput("Elevator/Volts", volts);
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
}
