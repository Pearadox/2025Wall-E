// Copyright 2021-2025 FRC 6328
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

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
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

  public static class VisionConstants {
    public static final String LL_NAME = "limelight-walle";

    public static final Vector<N3> LIMELIGHT_STD_DEV = VecBuilder.fill(.7, .7, .9999999);
    public static final Vector<N3> MEGATAG2_LIMELIGHT_STD_DEV = VecBuilder.fill(.7, .7, .9999999);
    
    public static final double AMBIGUITY_FILTER = 0.3;
    public static final double DISTANCE_FILTER = FieldConstants.FIELD_LENGTH / 2;

    // TODO: determine questnav std dev (likely lower than this)
    public static final Vector<N3> QUESTNAV_STD_DEV = VecBuilder.fill(.7, .7, .9999999);

    public static AprilTagFieldLayout aprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
  }

  public static final class FieldConstants {
    public static final double FIELD_LENGTH = Units.inchesToMeters(690.876);
    public static final double FIELD_WIDTH = Units.inchesToMeters(317);

    public static final int[] BLUE_REEF_TAG_IDS = {18, 19, 20, 21, 17, 22};
    public static final int[] BLUE_CORAL_STATION_TAG_IDS = {12, 13};
    public static final int[] RED_REEF_TAG_IDS = {7, 6, 11, 10, 9, 8};
    public static final int[] RED_CORAL_STATION_TAG_IDS = {1, 2};
  }
}
