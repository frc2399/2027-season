package first.robot.subsystems.drive;

import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.MetersPerSecondPerSecond;
import static org.wpilib.units.Units.RadiansPerSecond;

import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.LinearAcceleration;
import org.wpilib.units.measure.LinearVelocity;
import first.robot.SubsystemFactory;

public final class DriveConfig {
    public static final double kS;
    public static final double kV;
    public static final double DRIVE_P;
    public static final double DRIVE_D;
    public static final double TURN_P;
    public static final double TURN_D;
    public static final double HEADING_P;
    public static final double HEADING_D;
    public static final int PINION_TEETH;
    public static final Distance TRACK_WIDTH;
    public static final Distance TRACK_LENGTH;
    public static final LinearAcceleration MAX_ACCELERATION;
    public static final LinearVelocity MAX_SPEED;
    public static final AngularVelocity MAX_ANGULAR_VELOCITY;

    static {
        switch (SubsystemFactory.getRobotType()) {
            case BUBBLES:
                kS = 0.05;
                kV = 2.29;
                DRIVE_P = 0.4;
                DRIVE_D = 0;
                TURN_P = 1.0;
                TURN_D = 0.001;
                HEADING_P = 1.1;
                HEADING_D = 0.05;
                PINION_TEETH = 12;
                TRACK_WIDTH = Inches.of(24.5);
                TRACK_LENGTH = Inches.of(26.5);
                MAX_ACCELERATION = MetersPerSecondPerSecond.of(9.81);
                MAX_SPEED = MetersPerSecond.of(4.3);
                MAX_ANGULAR_VELOCITY = RadiansPerSecond.of(2 * Math.PI);
                break;
            case MOZART:
                kS = 0.1;
                kV = 2.29;
                DRIVE_P = 0.4;
                DRIVE_D = 0;
                TURN_P = 1.0;
                TURN_D = 0.001;
                HEADING_P = 1.1;
                HEADING_D = 0.05;
                PINION_TEETH = 12;
                TRACK_WIDTH = Inches.of(26 - (2 * 1.75));
                TRACK_LENGTH = Inches.of(26 - (2 * 1.75));
                MAX_ACCELERATION = MetersPerSecondPerSecond.of(9.81); // TODO: characterize
                MAX_SPEED = MetersPerSecond.of(4.3);
                MAX_ANGULAR_VELOCITY = RadiansPerSecond.of(2 * Math.PI);
                break;
            case BETA:
                kS = 0.1;
                kV = 2.17;
                DRIVE_P = 0.2;
                DRIVE_D = 0;
                TURN_P = 1.0;
                TURN_D = 0.001;
                HEADING_P = 2;
                HEADING_D = 0.05;
                PINION_TEETH = 13;
                TRACK_WIDTH = Inches.of(20.5);
                TRACK_LENGTH = Inches.of(26.5);
                MAX_ACCELERATION = MetersPerSecondPerSecond.of(9.81);
                MAX_SPEED = MetersPerSecond.of(4.3);
                MAX_ANGULAR_VELOCITY = RadiansPerSecond.of(2 * Math.PI);
                break;
            case COMP:
                kS = 0.15;
                kV = 2.23;
                DRIVE_P = 0.35;
                DRIVE_D = 0;
                TURN_P = 1.0;
                TURN_D = 0.001;
                HEADING_P = 2;
                HEADING_D = 0.05;
                PINION_TEETH = 13;
                TRACK_WIDTH = Inches.of(20.5);
                TRACK_LENGTH = Inches.of(26.5);
                MAX_ACCELERATION = MetersPerSecondPerSecond.of(9.81);
                MAX_SPEED = MetersPerSecond.of(4.3);
                MAX_ANGULAR_VELOCITY = RadiansPerSecond.of(2 * Math.PI);
                break;
            case SIM:
                kS = 0.1;
                kV = 2.17;
                DRIVE_P = 0.2;
                DRIVE_D = 0;
                TURN_P = 1.0;
                TURN_D = 0.001;
                HEADING_P = 2;
                HEADING_D = 0.05;
                PINION_TEETH = 13;
                TRACK_WIDTH = Inches.of(20.5);
                TRACK_LENGTH = Inches.of(26.5);
                MAX_ACCELERATION = MetersPerSecondPerSecond.of(9.81);
                MAX_SPEED = MetersPerSecond.of(4.3);
                MAX_ANGULAR_VELOCITY = RadiansPerSecond.of(2 * Math.PI);
                break;
            default:
                kS = 0;
                kV = 0;
                DRIVE_P = 0;
                DRIVE_D = 0;
                TURN_P = 0;
                TURN_D = 0;
                HEADING_P = 0;
                HEADING_D = 0;
                PINION_TEETH = 0;
                TRACK_WIDTH = Inches.of(0);
                TRACK_LENGTH = Inches.of(0);
                MAX_ACCELERATION = MetersPerSecondPerSecond.of(0);
                MAX_SPEED = MetersPerSecond.of(0);
                MAX_ANGULAR_VELOCITY = RadiansPerSecond.of(0);
                break;
        }
    }
}
