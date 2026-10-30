package frc.robot.subsystems.drive;

import static org.wpilib.units.Units.KilogramSquareMeters;
import static org.wpilib.units.Units.Kilograms;

import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.system.plant.DCMotor;
import org.wpilib.units.measure.Mass;
import org.wpilib.units.measure.MomentOfInertia;

import frc.robot.SubsystemFactory;

public final class PathPlannerConfig {
    public static final Mass ROBOT_MASS;
    public static final MomentOfInertia ROBOT_MOI;
    public static final double WHEEL_RADIUS; // meters
    public static final double MAX_SPEED; // meters per second
    public static final double WHEEL_COF;
    public static final DCMotor DRIVE_MOTOR;
    public static final double DRIVE_CURRENT_LIMIT; // amps
    public static final int NUM_MOTORS;
    public static final Translation2d MODULE_OFFSETS;

    static {
        switch (SubsystemFactory.getRobotType()) {
            case BUBBLES:
                ROBOT_MASS = Kilograms.of(0);
                ROBOT_MOI = KilogramSquareMeters.of(6.883);
                WHEEL_RADIUS = 0;
                MAX_SPEED = 4.3;
                WHEEL_COF = 0;
                DRIVE_MOTOR = null;
                DRIVE_CURRENT_LIMIT = 0;
                NUM_MOTORS = 0;
                MODULE_OFFSETS = new Translation2d();
                break;
            case MOZART:
                ROBOT_MASS = Kilograms.of(0);
                ROBOT_MOI = KilogramSquareMeters.of(0);
                WHEEL_RADIUS = 0;
                MAX_SPEED = 0;
                WHEEL_COF = 0;
                DRIVE_MOTOR = null;
                DRIVE_CURRENT_LIMIT = 0;
                NUM_MOTORS = 0;
                MODULE_OFFSETS = new Translation2d();
                break;
            case BETA:
                ROBOT_MASS = Kilograms.of(0);
                ROBOT_MOI = KilogramSquareMeters.of(0);
                WHEEL_RADIUS = 0;
                MAX_SPEED = 0;
                WHEEL_COF = 0;
                DRIVE_MOTOR = null;
                DRIVE_CURRENT_LIMIT = 0;
                NUM_MOTORS = 0;
                MODULE_OFFSETS = new Translation2d();
                break;
            case COMP:
                ROBOT_MASS = Kilograms.of(0);
                ROBOT_MOI = KilogramSquareMeters.of(0);
                WHEEL_RADIUS = 0;
                MAX_SPEED = 0;
                WHEEL_COF = 0;
                DRIVE_MOTOR = null;
                DRIVE_CURRENT_LIMIT = 0;
                NUM_MOTORS = 0;
                MODULE_OFFSETS = new Translation2d();
                break;
            // case SIM:
            //     kS = 0;
            //     kV = 0;
            //     DRIVE_P = 0;
            //     DRIVE_D = 0;
            //     TURN_P = 0;
            //     TURN_D = 0;
            //     HEADING_P = 0;
            //     HEADING_D = 0;
            //     PINION_TEETH = 0;
            //     TRACK_WIDTH = Inches.of(0);
            //     TRACK_LENGTH = Inches.of(0);
            //     MAX_ACCELERATION = MetersPerSecondPerSecond.of(0);
            //     MAX_SPEED = MetersPerSecond.of(0);
            //     MAX_ANGULAR_VELOCITY = RadiansPerSecond.of(0);
            //     break;
            default:
                ROBOT_MASS = Kilograms.of(0);
                ROBOT_MOI = KilogramSquareMeters.of(0);
                WHEEL_RADIUS = 0;
                MAX_SPEED = 0;
                WHEEL_COF = 0;
                DRIVE_MOTOR = null;
                DRIVE_CURRENT_LIMIT = 0;
                NUM_MOTORS = 0;
                MODULE_OFFSETS = new Translation2d();
                break;
        }
    }
}
