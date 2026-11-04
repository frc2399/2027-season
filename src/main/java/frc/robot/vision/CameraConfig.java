package frc.robot.vision;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.robot.SubsystemFactory;

public class CameraConfig {
    public static final String LIMELIGHT_NAME;
    public static final Angle CAMERA_PITCH; // 0 = horizontal; positive = leaning back
    public static final Angle CAMERA_YAW;
    public static final Distance X_ROBOT_TO_CAMERA_OFFSET; // positive = in front of robot center
    public static final Distance Y_ROBOT_TO_CAMERA_OFFSET; // positive = left of robot centerline
    public static final Distance Z_ROBOT_TO_CAMERA_OFFSET; // GROUND plane = 0 (not bellypan)
    public static final boolean HAS_SECOND_CAMERA;
    public static final String SECOND_LIMELIGHT_NAME;
    public static final Angle SECOND_CAMERA_PITCH; // 0 = horizontal; positive = leaning back
    public static final Angle SECOND_CAMERA_YAW;
    public static final Distance
            SECOND_X_ROBOT_TO_CAMERA_OFFSET; // positive = in front of robot center
    public static final Distance
            SECOND_Y_ROBOT_TO_CAMERA_OFFSET; // positive = left of robot centerline
    public static final Distance SECOND_Z_ROBOT_TO_CAMERA_OFFSET; // GROUND plane = 0 (not bellypan)

    static {
        switch (SubsystemFactory.getRobotType()) {
            case MOZART:
                LIMELIGHT_NAME = "";
                CAMERA_PITCH = Degrees.of(15);
                CAMERA_YAW = Degrees.of(153);
                X_ROBOT_TO_CAMERA_OFFSET = Inches.of(9.75);
                Y_ROBOT_TO_CAMERA_OFFSET = Inches.of(9.25);
                Z_ROBOT_TO_CAMERA_OFFSET = Inches.of(10);
                HAS_SECOND_CAMERA = false;
                SECOND_LIMELIGHT_NAME = "not_a_limelight";
                SECOND_CAMERA_PITCH = Degrees.of(0);
                SECOND_CAMERA_YAW = Degrees.of(0);
                SECOND_X_ROBOT_TO_CAMERA_OFFSET = Inches.of(0);
                SECOND_Y_ROBOT_TO_CAMERA_OFFSET = Inches.of(0);
                SECOND_Z_ROBOT_TO_CAMERA_OFFSET = Inches.of(0);
                break;
            case BUBBLES:
                LIMELIGHT_NAME = "";
                CAMERA_PITCH = Degrees.of(25);
                CAMERA_YAW = Degrees.of(0);
                X_ROBOT_TO_CAMERA_OFFSET = Inches.of(11.29);
                Y_ROBOT_TO_CAMERA_OFFSET = Inches.of(0);
                Z_ROBOT_TO_CAMERA_OFFSET = Inches.of(6.91);
                HAS_SECOND_CAMERA = false;
                SECOND_LIMELIGHT_NAME = "not_a_limelight";
                SECOND_CAMERA_PITCH = Degrees.of(0);
                SECOND_CAMERA_YAW = Degrees.of(0);
                SECOND_X_ROBOT_TO_CAMERA_OFFSET = Inches.of(0);
                SECOND_Y_ROBOT_TO_CAMERA_OFFSET = Inches.of(0);
                SECOND_Z_ROBOT_TO_CAMERA_OFFSET = Inches.of(0);
                break;
            case BETA:
                LIMELIGHT_NAME = "";
                CAMERA_PITCH = Degrees.of(23.5);
                CAMERA_YAW = Degrees.of(10);
                X_ROBOT_TO_CAMERA_OFFSET = Inches.of(12.5);
                Y_ROBOT_TO_CAMERA_OFFSET = Inches.of(-9.75);
                Z_ROBOT_TO_CAMERA_OFFSET = Inches.of(28.75);
                HAS_SECOND_CAMERA = false;
                SECOND_LIMELIGHT_NAME = "not_a_limelight";
                SECOND_CAMERA_PITCH = Degrees.of(0);
                SECOND_CAMERA_YAW = Degrees.of(0);
                SECOND_X_ROBOT_TO_CAMERA_OFFSET = Inches.of(0);
                SECOND_Y_ROBOT_TO_CAMERA_OFFSET = Inches.of(0);
                SECOND_Z_ROBOT_TO_CAMERA_OFFSET = Inches.of(0);
                break;
            case COMP:
                LIMELIGHT_NAME = "";
                CAMERA_PITCH = Degrees.of(32.794);
                CAMERA_YAW = Degrees.of(9.937);
                X_ROBOT_TO_CAMERA_OFFSET = Inches.of(12.479);
                Y_ROBOT_TO_CAMERA_OFFSET = Inches.of(9.505);
                Z_ROBOT_TO_CAMERA_OFFSET = Inches.of(20.751);
                HAS_SECOND_CAMERA = false;
                SECOND_LIMELIGHT_NAME = "not_a_limelight";
                SECOND_CAMERA_PITCH = Degrees.of(0);
                SECOND_CAMERA_YAW = Degrees.of(0);
                SECOND_X_ROBOT_TO_CAMERA_OFFSET = Inches.of(0);
                SECOND_Y_ROBOT_TO_CAMERA_OFFSET = Inches.of(0);
                SECOND_Z_ROBOT_TO_CAMERA_OFFSET = Inches.of(0);
                break;
            default:
                LIMELIGHT_NAME = "";
                CAMERA_PITCH = Degrees.of(0);
                CAMERA_YAW = Degrees.of(0);
                X_ROBOT_TO_CAMERA_OFFSET = Inches.of(0);
                Y_ROBOT_TO_CAMERA_OFFSET = Inches.of(0);
                Z_ROBOT_TO_CAMERA_OFFSET = Inches.of(0);
                HAS_SECOND_CAMERA = false;
                SECOND_LIMELIGHT_NAME = "not_a_limelight";
                SECOND_CAMERA_PITCH = Degrees.of(0);
                SECOND_CAMERA_YAW = Degrees.of(0);
                SECOND_X_ROBOT_TO_CAMERA_OFFSET = Inches.of(0);
                SECOND_Y_ROBOT_TO_CAMERA_OFFSET = Inches.of(0);
                SECOND_Z_ROBOT_TO_CAMERA_OFFSET = Inches.of(0);
        }
    }
}
