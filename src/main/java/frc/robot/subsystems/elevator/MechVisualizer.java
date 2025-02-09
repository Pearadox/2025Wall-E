package frc.robot.subsystems.elevator;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants.SimulationConstants;
import frc.robot.subsystems.elevator.ArmSim.ArmState;

public class MechVisualizer {
    private final Mechanism2d mech2d = new Mechanism2d(Units.inchesToMeters(45), Units.inchesToMeters(90));
    private final MechanismRoot2d elevatorRoot =
            mech2d.getRoot("Elevator Root", Units.inchesToMeters(22.5), Units.inchesToMeters(0.5));
    private final MechanismLigament2d elevator2d =
            elevatorRoot.append(new MechanismLigament2d("Elevator", SimulationConstants.MIN_HEIGHT, 90));
    private MechanismLigament2d arm = elevator2d.append(
            new MechanismLigament2d("Arm", SimulationConstants.ARM_LENGTH, 0, 5, new Color8Bit(Color.kYellow)));
    private MechanismLigament2d cam = arm.append(new MechanismLigament2d(
            "CAM", SimulationConstants.CAM_LENGTH, 0, 3, new Color8Bit(Color.kMediumSpringGreen)));

    private InterpolatingDoubleTreeMap camLerp = new InterpolatingDoubleTreeMap();

    private static final MechVisualizer instance = new MechVisualizer();

    public static MechVisualizer getInstance() {
        return instance;
    }

    private MechVisualizer() {
        camLerp.put(ArmState.L4.angle, -55.0);
        camLerp.put(ArmState.L3.angle, -55.0);
        camLerp.put(ArmState.L2.angle, -55.0);
        camLerp.put(ArmState.CoralStation.angle, -180 - 35.0);
        camLerp.put(-90.0, -90.0);
        // camLerp.put(ArmState.L4.angle, 0.0);
        // camLerp.put(ArmState.L3.angle, 0.0);
        // camLerp.put(ArmState.L2.angle, 90.0);
        // camLerp.put(ArmState.CoralStation.angle, -90.0);
        // camLerp.put(-180.0, -180.0);
        // camLerp.put(180.0, 180.0);
    }

    public void periodic() {
        SmartDashboard.putData("Elevator Sim", mech2d);
    }

    public void updateElevatorHeight(double heightMeters) {
        elevator2d.setLength(heightMeters);
        ProjectileIntakeSim.getInstance().updateElevatorHeight(heightMeters);
    }

    public void updateArmAngle(double angleRads) {
        double angleDegs = Units.radiansToDegrees(angleRads) - 90;
        arm.setAngle(angleDegs);
        cam.setAngle(-angleDegs + camLerp.get(angleDegs + 90) - 90);
        ProjectileIntakeSim.getInstance().updateArmAngle(Units.degreesToRadians(angleDegs));
    }
}
