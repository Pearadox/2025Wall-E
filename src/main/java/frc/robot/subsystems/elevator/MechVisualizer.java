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
        camLerp.put(-40.0, (90 - 35) - 180.0);
        camLerp.put(-45 - 90.0, (90 - 35) - 180.0);
        camLerp.put(-45 - 90.0, (90 - 35) - 180.0);
        camLerp.put(110.0, 55.0);
        camLerp.put(-180.0, -180.0);
        camLerp.put(180.0, 180.0);
    }

    public void periodic() {
        SmartDashboard.putData("Elevator Sim", mech2d);
    }

    public void updateElevatorHeight(double heightMeters) {
        elevator2d.setLength(heightMeters);
        ProjectileIntakeSim.getInstance().updateElevatorHeight(heightMeters);
    }

    public void updateArmAngle(double angleRads) {
        double armAngle = Units.radiansToDegrees(angleRads) - 90;
        arm.setAngle(armAngle);
        cam.setAngle(-armAngle + camLerp.get(armAngle));
        ProjectileIntakeSim.getInstance().updateArmAngle(angleRads);
    }
}
