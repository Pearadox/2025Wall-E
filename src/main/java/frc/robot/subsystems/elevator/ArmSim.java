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
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drivers.PearadoxTalonFX;
import frc.robot.Constants.SimulationConstants;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;

public class ArmSim extends SubsystemBase {
    private DCMotor armGearbox = DCMotor.getKrakenX60(1);

    private SingleJointedArmSim armSim = new SingleJointedArmSim(
            armGearbox,
            SimulationConstants.ARM_GEARING,
            SimulationConstants.ARM_MOI,
            SimulationConstants.ARM_LENGTH,
            SimulationConstants.MIN_ANGLE,
            SimulationConstants.MAX_ANGLE,
            SimulationConstants.SIMULATE_GRAVITY,
            SimulationConstants.STARTING_ANGLE);

    private PearadoxTalonFX pivot;
    private TalonFXSimState pivotSim;

    public double setpointRotations;

    public enum ArmState {
        L4(50.0),
        L3(-45.0),
        L2(-45.0),
        CoralStation(-160.0),
        Stowed(-90.0);

        public final double angle;

        private ArmState(double angle) {
            this.angle = angle;
        }
    }

    public static final ArmSim ARMSIM = new ArmSim();

    public static ArmSim getInstance() {
        return ARMSIM;
    }

    public ArmSim() {
        pivot = new PearadoxTalonFX(0, NeutralModeValue.Brake, 80, true);
        pivotSim = new TalonFXSimState(pivot);

        Slot0Configs slot0Configs = new Slot0Configs();
        slot0Configs.kP = 0.75;
        slot0Configs.kG = 0;
        slot0Configs.kI = 0.1;
        slot0Configs.kD = 0;
        pivot.getConfigurator().apply(slot0Configs);
    }

    public void simulationPeriodic() {
        // This method will be called once per scheduler run
        armUp();

        SmartDashboard.putNumber("Position", pivot.getPosition().getValueAsDouble());

        armSim.setInput(pivotSim.getMotorVoltage());
        SmartDashboard.putNumber("Voltage", pivotSim.getMotorVoltage());

        armSim.update(0.02);

        SmartDashboard.putNumber("Angle", armSim.getAngleRads());
        SmartDashboard.putNumber("Angle Velocity", armSim.getVelocityRadPerSec());

        pivotSim.setSupplyVoltage(12);

        pivotSim.setRawRotorPosition((Units.radiansToRotations(armSim.getAngleRads())
                        - Units.degreesToRotations(SimulationConstants.STARTING_ANGLE))
                * SimulationConstants.ARM_GEARING);
        pivotSim.setRotorVelocity(
                Units.radiansToRotations(armSim.getVelocityRadPerSec() * SimulationConstants.ARM_GEARING));

        MechVisualizer.getInstance().updateArmAngle(armSim.getAngleRads());
    }

    public void armUp() {
        final PositionVoltage request = new PositionVoltage(setpointRotations).withSlot(0);
        pivot.setControl(request);
    }

    public void setSetpoint(ArmState armState) {
        setpointRotations = Units.degreesToRotations(-armState.angle) * SimulationConstants.ARM_GEARING;
    }

    public void shootCoral(SwerveDriveSimulation driveSimulation) {
        ProjectileIntakeSim.getInstance().shootCoral();
    }
}
