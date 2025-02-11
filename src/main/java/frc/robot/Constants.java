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

        public static final double ELEVATOR_GEARING = 15; // TODO
        public static final double CARRIAGE_MASS = Units.lbsToKilograms(28) + ARM_MASS; // TODO
        public static final double DRUM_RADIUS = Units.inchesToMeters(0.955 / 2); // TODO
        public static final double MIN_HEIGHT = Units.inchesToMeters(45 - 6);
        public static final double MAX_HEIGHT = Units.inchesToMeters(72 + 6);
        public static final double STARTING_HEIGHT = MIN_HEIGHT;
    }

    public static class FieldConstants {
        public static final int[] BLUE_REEF_TAG_IDS = {18, 19, 20, 21, 17, 22};
        public static final int[] BLUE_CORAL_STATION_TAG_IDS = {12, 13};

        public static final int[] RED_REEF_TAG_IDS = {7, 6, 11, 10, 9, 8};
        public static final int[] RED_CORAL_STATION_TAG_IDS = {1, 2};
    }
}
