// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

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

public class ElevatorIOSim extends SubsystemBase {

    private DCMotor armGearbox = DCMotor.getKrakenX60(1);

    private SingleJointedArmSim armSim = new SingleJointedArmSim(
            armGearbox,
            3,
            SingleJointedArmSim.estimateMOI(Units.inchesToMeters(30), 8),
            Units.inchesToMeters(30),
            Units.degreesToRadians(-75),
            Units.degreesToRadians(255),
            false,
            0,
            2.0 * Math.PI / 4096,
            0.0);

    private Mechanism2d mech2d = new Mechanism2d(60, 60);
    private MechanismRoot2d armPivot = mech2d.getRoot("Arm Pivot", 30, 30);
    private MechanismLigament2d elevator = armPivot.append(new MechanismLigament2d("Elevator", 30, -90));
    private MechanismLigament2d arm =
            armPivot.append(new MechanismLigament2d("Arm", 30, 0, 6, new Color8Bit(Color.kYellow)));

    /** Creates a new ElevatorIOSim. */
    public ElevatorIOSim() {}

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putData("Arm Sim", mech2d);

        arm.setAngle(Units.radiansToDegrees(armSim.getAngleRads()));
    }
} 
