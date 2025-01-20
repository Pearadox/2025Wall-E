package frc.robot.util;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.FloatArraySubscriber;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;

public class QuestNav {
    // Configure Network Tables topics (questnav/...) to communicate with the Quest HMD
    private static NetworkTableInstance nt4Instance = NetworkTableInstance.getDefault();
    private static NetworkTable nt4Table = nt4Instance.getTable("questnav");
    private static IntegerSubscriber questMiso = nt4Table.getIntegerTopic("miso").subscribe(0);
    private static IntegerPublisher questMosi = nt4Table.getIntegerTopic("mosi").publish();

    private static IntegerSubscriber questFrameCount =
            nt4Table.getIntegerTopic("frameCount").subscribe(0);

    private static DoubleSubscriber questTimestamp =
            nt4Table.getDoubleTopic("timestamp").subscribe(0.0f);

    private static FloatArraySubscriber questPosition =
            nt4Table.getFloatArrayTopic("position").subscribe(new float[] {0.0f, 0.0f, 0.0f});

    private static FloatArraySubscriber questQuaternion =
            nt4Table.getFloatArrayTopic("quaternion")
                    .subscribe(new float[] {0.0f, 0.0f, 0.0f, 0.0f});

    private static FloatArraySubscriber questEulerAngles =
            nt4Table.getFloatArrayTopic("eulerAngles").subscribe(new float[] {0.0f, 0.0f, 0.0f});

    private static DoubleSubscriber questBattery =
            nt4Table.getDoubleTopic("batteryPercent").subscribe(0.0f);

    // // publishers
    // private static DoubleArrayPublisher questOdometry =
    //         nt4Table.getDoubleArrayTopic("odometry").publish();
    
    // private static StructPublisher<Pose2d> questOdometryStruct =
    //         NetworkTableInstance.getDefault()
    //                 .getStructTopic("QuestOdometry", Pose2d.struct)
    //                 .publish();
    
    // private static DoubleArrayPublisher questOdometryOffset =
    //     nt4Table.getDoubleArrayTopic("odometryOffset").publish();
    
    // private static final DoubleArrayPublisher questResetPose = nt4Table.getDoubleArrayTopic("resetpose").publish();

    // Constant mounting offset
    // TODO: find this by rotating the robot 180 degrees and dividing the quest pose x and y by 2
    private static final Transform2d ROBOT_TO_QUEST = new Transform2d();

    // Current robot pose offset
    private static Translation2d questTranslationOffset = new Translation2d();
    private static double yawOffset = 0.0;

    public static void periodic() {
        // publishOdometry();
        cleanUpMessages();

        Logger.recordOutput("QuestNav/Quest Pose", getQuestNavPose());
        Logger.recordOutput("QuestNav/Robot Pose", getRobotPose());
        Logger.recordOutput("QuestNav/Frame Count", questFrameCount.get());
        Logger.recordOutput("QuestNav/Timestamp", questTimestamp.get());
        Logger.recordOutput("QuestNav/Raw Position", questPosition.get());
        Logger.recordOutput("QuestNav/Offset", getOffset());
        Logger.recordOutput("QuestNav/Quaternion", getQuaternion());
        Logger.recordOutput("QuestNav/Battery", questBattery.get()); 
        Logger.recordOutput("QuestNav/Connected", isConnected()); 
    }
    
    // Clean up QuestNav subroutine messages after processing on the headset
    public static void cleanUpMessages() {
        if (questMiso.get() == 99) {
            questMosi.set(0);
        }
    }

    public static void resetQuestPose(Pose2d robotPose) {
        zeroQuestPose();
        resetHeading(robotPose.getRotation().getDegrees());
        resetTranslation(robotPose.getTranslation());
    }

    // equivalent to long pressing the quest button; its pose goes to 0
    public static void zeroQuestPose() {
        if (questMiso.get() != 99) {
            questMosi.set(1);
        }
    }

    public static void resetTranslation(Translation2d translation) {
        questTranslationOffset = translation;
    }

    public static void resetHeading(double angleDegrees) {
        float[] eulerAngles = questEulerAngles.get();
        yawOffset = eulerAngles[1] + angleDegrees;
    }

    public static Translation2d getQuestNavPosition() {
        float[] questNavPosition = questPosition.get();
        Translation2d rawTranslation = new Translation2d(questNavPosition[2], -questNavPosition[0]);
        return rawTranslation.plus(questTranslationOffset);
    }

    public static double getQuestNavYaw() {
        float[] eulerAngles = questEulerAngles.get();
        double ret = eulerAngles[1] - yawOffset;
        ret %= 360;
        if (ret < 0) {
            ret += 360;
        }
        return 360 - ret;
    }

    public static Pose2d getQuestNavPose() {
        return new Pose2d(getQuestNavPosition(), Rotation2d.fromDegrees(getQuestNavYaw()));
    }

    public static Pose2d getRobotPose() {
        return getQuestNavPose().transformBy(ROBOT_TO_QUEST.inverse());
    }

    public static Transform2d getOffset() {
        return new Transform2d(questTranslationOffset, Rotation2d.fromDegrees(yawOffset));
    }

    public static Quaternion getQuaternion() {
        float[] quaternion = questQuaternion.get();
        return new Quaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3]);
    }

    // Returns if the Quest is connected.
    public static boolean isConnected() {
        return ((RobotController.getFPGATime() - questBattery.getLastChange()) / 1000) < 250;
    }

    // public static void publishOdometry() {
    //     Pose2d pose = getQuestNavPose();
    //     double[] odometry = {pose.getX(), pose.getY(), pose.getRotation().getDegrees()};
    //     questOdometry.set(odometry);
    //     questOdometryStruct.set(pose);

    //     double[] odometryOffset = {questTranslationOffset.getX(), questTranslationOffset.getY(), yawOffset};
    //     questOdometryOffset.set(odometryOffset);
    // }    

    // // quest will begin reporting pose pre-transformed to robot
    // // potentially less useful for logging?
    // public static void setQuestPose(Pose2d robotPose) {
    //     Pose2d questPose = robotPose.plus(ROBOT_TO_QUEST);

    //     if (questMiso.get() != 99) {
    //         questResetPose.set(new double [] {
    //             questPose.getX(), questPose.getY(), questPose.getRotation().getDegrees()
    //         });
    //         questMosi.set(2);
    //     }

    //     questTranslationOffset = new Translation2d();
    //     yawOffset = 0.0;
    // }
}