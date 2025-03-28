// Copyright 2021-2024 FRC 6328
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

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running on a roboRIO. Change
 * the value of "simMode" to switch between "sim" (physics sim) and "replay" (log replay from a file).
 */
public final class Constants {
    public static final Mode simMode = Mode.SIM;
    public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

    public static enum Mode {
        /** Running on a real robot. */
        REAL,

        /** Running a physics simulator. */
        SIM,

        /** Replaying from a log file. */
        REPLAY
    }

    public static class SimulationConstants {
        public static final boolean SIMULATE_GRAVITY = true;

        public static final double ARM_GEARING = 15; // TODO
        public static final double ARM_MASS = Units.lbsToKilograms(4); // TODO
        public static final double ARM_LENGTH = Units.inchesToMeters(12);
        public static final double ARM_MOI = SingleJointedArmSim.estimateMOI(ARM_LENGTH, ARM_MASS); // TODO
        public static final double MIN_ANGLE = Units.degreesToRadians(-180); // TODO
        public static final double MAX_ANGLE = Units.degreesToRadians(180); // TODO
        public static final double STARTING_ANGLE = Units.degreesToRadians(-90); // TODO

        // joint of ee to bottom of coral
        public static final double CAM_LENGTH = Units.inchesToMeters(14.5);

        // joint of ee to top plane of coral
        public static final double EE_TO_CORAL_HEIGHT = Units.inchesToMeters(2.5);
        public static final double EE_TO_CORAL_WIDTH = Units.inchesToMeters(4.25);
        public static final double CORAL_LENGTH = Units.inchesToMeters(11.875);

        public static final double ELEVATOR_GEARING = 7.5; // TODO
        public static final double CARRIAGE_MASS = Units.lbsToKilograms(28 / 2) + ARM_MASS; // TODO
        public static final double DRUM_RADIUS = Units.inchesToMeters(0.955 / 2); // TODO
        public static final double MIN_HEIGHT = Units.inchesToMeters(45 - 6);
        public static final double MAX_HEIGHT = Units.inchesToMeters(72 + 6);
        public static final double STARTING_HEIGHT = MIN_HEIGHT;

        // new EE viz
        public static final double PIVOT_TO_MIDDLE_OF_CORAL_ANG_OFFSET = Units.degreesToRadians(-20.2531597269);
        public static final double PIVOT_TO_MIDDLE_OF_CORAL_RADIUS = Units.inchesToMeters(23.249031544);
        public static final double PIVOT_ANGLE_TO_CORAL_ANGLE = Units.degreesToRadians(243.986);
    }

    public static class FieldConstants {
        public static final int[] BLUE_REEF_TAG_IDS = {18, 19, 20, 21, 17, 22};
        public static final int[] BLUE_CORAL_STATION_TAG_IDS = {12, 13};

        public static final int[] RED_REEF_TAG_IDS = {7, 6, 11, 10, 9, 8};
        public static final int[] RED_CORAL_STATION_TAG_IDS = {1, 2};

        public static final double BRANCH_SPACING = Units.inchesToMeters(12.94 / 2.0);

        public static final Translation2d BLUE_REEF_CENTER = new Translation2d(4.5, 4);
        public static final Translation2d BLUE_NPS_CORAL_STATION =
                new Translation2d(Units.inchesToMeters(33.526), Units.inchesToMeters(291.176));
        public static final Translation2d BLUE_PS_CORAL_STATION =
                new Translation2d(Units.inchesToMeters(33.526), Units.inchesToMeters(25.824));

        // the top of the branch (L4) is ~2" behind the april tag
        public static final double BRANCH_OFFSET_BEHIND_APRILTAG = Units.inchesToMeters(2.049849);
        public static final double L4_HEIGHT = Units.inchesToMeters(72);
    }

    public static final class AlignConstants {
        // TODO: possible align pid adjustment
        // public static final double ALIGN_STRAFE_KP = 0.06;
        // public static final double ALIGN_STRAFE_KI = 0.001;
        // public static final double ALIGN_FORWARD_KP = 0.04; // -0.06
        public static final double ALIGN_KS = 0.09;

        // tx and ty tolerances with setpoint
        public static final double ALIGN_TOLERANCE_PIXELS = 0.5;
        // don't try translationally aligning unless rotation is already aligned within this tolerance
        public static final double ALIGN_ROT_TOLERANCE_DEGREES = 10;

        // reduce speed by 1/4 every tick when an april tag is not seen
        public static final double ALIGN_DAMPING_FACTOR = 0.75;
        public static final double ALIGN_SPEED_DEADBAND = 0.025;

        public static final double BRANCH_SPACING = Units.inchesToMeters(12.97 / 2.0); // 12.94 //12.97

        // target relative
        public static final double REEF_ALIGN_MID_TX = 0; // 0.28575
        public static final double REEF_ALIGN_LEFT_TX = -BRANCH_SPACING + REEF_ALIGN_MID_TX - 0.05;
        public static final double REEF_ALIGN_RIGHT_TX = BRANCH_SPACING + REEF_ALIGN_MID_TX + 0.04 - 0.05;
        public static final double REEF_ALIGN_TZ = 0.5; // target relative

        public static final double STATION_ALIGN_TX = 0.07;
        public static final double STATION_ALIGN_TZ = -0.3;

        public static final double REEF_kP = 0.5; // Tune all PID values
        public static final double REEF_kI = 0;
        public static final double REEF_kD = 0;

        public static final double REEF_Forward_kP = 0.2 * 5; // Tune all PID values

        public static final double ROT_REEF_kP = 0.02; // Tune all PID values
        public static final double ROT_REEF_kI = 0;
        public static final double ROT_REEF_kD = 0;

        public static final PathConstraints PATH_CONSTRAINTS =
                new PathConstraints(3.0, 4.0, Units.degreesToRadians(540), Units.degreesToRadians(720));
    }

    public class ArmConstants {
        public static final double PIVOT_TO_CORAL_RADIUS = Units.inchesToMeters(23.4106654653);
        public static final double ARM_TO_CORAL_ANGULAR_OFFSET = Units.degreesToRadians(34.8693502919);
    }
}
