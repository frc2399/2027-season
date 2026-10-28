package first.robot.subsystems.drive;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.Milliseconds;
import static org.wpilib.units.Units.RotationsPerSecond;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import org.wpilib.math.util.MathUtil;
import org.wpilib.math.util.Units;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.LinearVelocity;
import org.wpilib.telemetry.Telemetry;
import first.robot.RobotConstants;
import first.robot.constants.RobotConstants.MotorConstants;

public class SwerveModuleHardwareVortex implements SwerveModuleIO {

    private SparkFlex drivingSparkFlex;
    private SparkMax turningSparkMax;

    private final RelativeEncoder drivingRelativeEncoder;
    private final SparkAbsoluteEncoder turningAbsoluteEncoder;

    private final SparkClosedLoopController drivingPidController;
    private final SparkClosedLoopController turningPidController;

    private double chassisAngularOffset;
    private String name;
    private double desiredAngle;
    private double driveDesiredVelocity;
    private static final SparkFlexConfig sparkFlexConfigDriving = new SparkFlexConfig();
    private static final ClosedLoopConfig sparkFlexClosedLoopConfigDriving = new ClosedLoopConfig();
    private static final SparkMaxConfig sparkMaxConfigTurning = new SparkMaxConfig();
    private static final ClosedLoopConfig sparkMaxClosedLoopConfigTurning = new ClosedLoopConfig();

    private boolean isOptimizedBackwards = false;

    // drivings are NEO Vortex, turnings are NEO 550s
    private static int DRIVING_MOTOR_PINION_TEETH;

    // Invert the turning encoder, since the output shaft rotates in the opposite
    // direction of the steering motor in the MAXSwerve Module.
    private static final boolean TURNING_ENCODER_INVERTED = true;
    private static final boolean DRIVING_MOTOR_INVERTED = false;
    private static final boolean TURNING_MOTOR_INVERTED = false;

    // Calculations required for driving motor conversion factors and feed forward
    private static final Distance WHEEL_DIAMETER = Inches.of(1.5 * 2);
    private static final Distance WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER.times(Math.PI);

    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    // This is also the gear ratio (14T)

    private final double DRIVING_MOTOR_REDUCTION;

    private final LinearVelocity DRIVE_WHEEL_FREE_SPEED;

    private final Distance DRIVING_ENCODER_POSITION_FACTOR; // meters
    private final Distance DRIVING_ENCODER_VELOCITY_FACTOR; // meters
    // per
    // second

    private static final double TURNING_ENCODER_POSITION_FACTOR = Units.rotationsToRadians(1);
    private static final double TURNING_ENCODER_VELOCITY_FACTOR =
            Units.rotationsToRadians(1) / 60.0;

    private static final boolean TURNING_ENCODER_POSITION_WRAPPING = true;
    private static final double TURNING_ENCODER_POSITION_PID_MIN_INPUT = 0; // radians
    private static final double TURNING_ENCODER_POSITION_PID_MAX_INPUT =
            TURNING_ENCODER_POSITION_FACTOR; // radians

    private final double DRIVING_P;
    private final double DRIVING_I = 0;
    private final double DRIVING_D;
    private final double DRIVING_KS;
    private final double DRIVING_KV;
    private static final double DRIVING_MIN_OUTPUT = -1;

    private static final double DRIVING_MAX_OUTPUT = 1;

    private final double TURNING_P;
    private static final double TURNING_I = 0;
    private static double TURNING_D;
    private static final double DRIVING_KA = 0;
    private static final double TURNING_MIN_OUTPUT = -1;
    private static final double TURNING_MAX_OUTPUT = 1;
    private static final double TURNING_KS = 0;
    private static final double TURNING_KV = 0;
    private static final double TURNING_KA = 0;

    private static final double VOLTAGE_COMPENSATION = 12;

    private static final SparkBaseConfig.IdleMode DRIVING_MOTOR_IDLE_MODE =
            SparkBaseConfig.IdleMode.kBrake;
    private static final SparkBaseConfig.IdleMode TURNING_MOTOR_IDLE_MODE =
            SparkBaseConfig.IdleMode.kBrake;

    public SwerveModuleHardwareVortex(
            int drivingCanId, int turningCanId, double chassisAngularOffset, String name) {
        this.chassisAngularOffset = chassisAngularOffset;
        this.name = name;
        drivingSparkFlex = new SparkFlex(drivingCanId, MotorType.kBrushless);
        turningSparkMax = new SparkMax(turningCanId, MotorType.kBrushless);

        // get from config!
        DRIVING_KS = DriveConfig.kS;
        DRIVING_KV = DriveConfig.kV;
        DRIVING_P = DriveConfig.DRIVE_P;
        DRIVING_D = DriveConfig.DRIVE_D;
        TURNING_P = DriveConfig.TURN_P;
        TURNING_D = DriveConfig.TURN_D;
        DRIVING_MOTOR_PINION_TEETH = DriveConfig.PINION_TEETH;

        Telemetry.log("drive/config/ks", DRIVING_KS);
        Telemetry.log("drive/config/kv", DRIVING_KV);
        Telemetry.log("drive/config/swervedrivep", DRIVING_P);
        Telemetry.log("drive/config/swervedrived", DRIVING_D);
        Telemetry.log("drive/config/turnp", TURNING_P);
        Telemetry.log("drive/config/turnd", TURNING_D);
        Telemetry.log("drive/config/pinion", DRIVING_MOTOR_PINION_TEETH);

        // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
        // teeth on the bevel pinion
        // This is also the gear ratio (14T)
        // a lot of this had to get moved down here because pinion teeth is variable by
        // robot & assigned via module config
        DRIVING_MOTOR_REDUCTION = (45.0 * 22) / (DRIVING_MOTOR_PINION_TEETH * 15);
        DRIVE_WHEEL_FREE_SPEED =
                MetersPerSecond.of(
                        (MotorConstants.VORTEX_FREE_SPEED.in(RotationsPerSecond)
                                        * WHEEL_CIRCUMFERENCE.in(Meters))
                                / (DRIVING_MOTOR_REDUCTION));

        DRIVING_ENCODER_POSITION_FACTOR =
                (WHEEL_DIAMETER.times(Math.PI)).div(DRIVING_MOTOR_REDUCTION); // meters
        DRIVING_ENCODER_VELOCITY_FACTOR = DRIVING_ENCODER_POSITION_FACTOR.div(60); // meters
        // per
        // second

        sparkFlexConfigDriving
                .inverted(DRIVING_MOTOR_INVERTED)
                .idleMode(DRIVING_MOTOR_IDLE_MODE)
                .smartCurrentLimit((int) MotorConstants.VORTEX_CURRENT_LIMIT.in(Amps))
                .voltageCompensation(VOLTAGE_COMPENSATION);
        sparkFlexConfigDriving
                .encoder
                .positionConversionFactor(DRIVING_ENCODER_POSITION_FACTOR.in(Meters))
                .velocityConversionFactor(DRIVING_ENCODER_VELOCITY_FACTOR.in(Meters));
        sparkFlexConfigDriving
                .closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .outputRange(DRIVING_MIN_OUTPUT, DRIVING_MAX_OUTPUT);

        sparkFlexClosedLoopConfigDriving
                .pid(DRIVING_P, DRIVING_I, DRIVING_D)
                .feedForward
                .sva(DRIVING_KS, DRIVING_KV, DRIVING_KA);

        sparkFlexConfigDriving.apply(sparkFlexClosedLoopConfigDriving);

        sparkMaxConfigTurning
                .inverted(TURNING_MOTOR_INVERTED)
                .idleMode(TURNING_MOTOR_IDLE_MODE)
                .smartCurrentLimit((int) MotorConstants.NEO550_CURRENT_LIMIT.in(Amps))
                .voltageCompensation(VOLTAGE_COMPENSATION);
        sparkMaxConfigTurning
                .absoluteEncoder
                .positionConversionFactor(TURNING_ENCODER_POSITION_FACTOR)
                .velocityConversionFactor(TURNING_ENCODER_VELOCITY_FACTOR);
        sparkMaxConfigTurning.absoluteEncoder.inverted(TURNING_ENCODER_INVERTED);
        sparkMaxConfigTurning
                .closedLoop
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                .outputRange(TURNING_MIN_OUTPUT, TURNING_MAX_OUTPUT)
                .positionWrappingEnabled(TURNING_ENCODER_POSITION_WRAPPING)
                .positionWrappingInputRange(
                        TURNING_ENCODER_POSITION_PID_MIN_INPUT,
                        TURNING_ENCODER_POSITION_PID_MAX_INPUT);
        sparkMaxConfigTurning.signals.absoluteEncoderPositionPeriodMs(
                (int) RobotConstants.SpeedConstants.MAIN_LOOP_FREQUENCY.in(Milliseconds));

        sparkMaxClosedLoopConfigTurning
                .pid(TURNING_P, TURNING_I, TURNING_D)
                .feedForward
                .sva(TURNING_KS, TURNING_KV, TURNING_KA);

        sparkFlexConfigDriving.apply(sparkFlexClosedLoopConfigDriving);
        sparkMaxConfigTurning.apply(sparkMaxClosedLoopConfigTurning);

        var drivingStatus =
                drivingSparkFlex.configure(
                        sparkFlexConfigDriving,
                        ResetMode.kResetSafeParameters,
                        PersistMode.kPersistParameters);
        var turningStatus =
                turningSparkMax.configure(
                        sparkMaxConfigTurning,
                        ResetMode.kResetSafeParameters,
                        PersistMode.kPersistParameters);

        drivingRelativeEncoder = drivingSparkFlex.getEncoder();
        turningAbsoluteEncoder = turningSparkMax.getAbsoluteEncoder();

        drivingPidController = drivingSparkFlex.getClosedLoopController();
        turningPidController = turningSparkMax.getClosedLoopController();

        if (drivingStatus != REVLibError.kOk) {
            System.err.println("Failed to configure driving motor: " + name + " " + drivingStatus);
        }
        if (turningStatus != REVLibError.kOk) {
            System.err.println("Failed to configure turning motor: " + name + " " + turningStatus);
        }
    }

    public void setDriveEncoderPosition(double position) {
        drivingRelativeEncoder.setPosition(position);
    }
    ;

    public double getDriveEncoderPosition() {
        return drivingRelativeEncoder.getPosition();
    }

    public void setDesiredDriveSpeedMPS(double speed, boolean isFlipped) {
        drivingPidController.setSetpoint(speed, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
        this.driveDesiredVelocity = speed;
        this.isOptimizedBackwards = isFlipped;
    }
    ;

    public double getDriveEncoderSpeedMPS() {
        return drivingRelativeEncoder.getVelocity();
    }
    ;

    public double getTurnEncoderPosition() {
        return turningAbsoluteEncoder.getPosition();
    }
    ;

    public void setDesiredTurnAngle(double angle) {
        turningPidController.setSetpoint(angle, ControlType.kPosition, ClosedLoopSlot.kSlot0);
        this.desiredAngle = angle;
    }
    ;

    public double getChassisAngularOffset() {
        return chassisAngularOffset;
    }

    public void updateStates(SwerveModuleIOStates states) {
        states.desiredAngle = Units.radiansToDegrees(MathUtil.angleModulus(this.desiredAngle));
        states.driveEncoderPos = getDriveEncoderPosition();
        states.driveVoltage =
                drivingSparkFlex.getBusVoltage() * drivingSparkFlex.getAppliedOutput();
        states.turnVoltage = turningSparkMax.getBusVoltage() * turningSparkMax.getAppliedOutput();
        states.driveCurrent = drivingSparkFlex.getOutputCurrent();
        states.turnCurrent = turningSparkMax.getOutputCurrent();
        states.driveDesiredVelocity = this.driveDesiredVelocity;

        if (isOptimizedBackwards) {
            states.turnAngle = states.desiredAngle + Math.PI;
            states.driveVelocity = getDriveEncoderSpeedMPS();

        } else {
            states.turnAngle =
                    Units.radiansToDegrees(MathUtil.angleModulus(getTurnEncoderPosition()));
            states.driveVelocity = getDriveEncoderSpeedMPS() * -1;
        }

        Telemetry.log(
                "Swerve/module " + name + "/turn desired angle (deg)", states.desiredAngle);
        Telemetry.log("Swerve/module " + name + "/turn angle (deg)", states.turnAngle);
        Telemetry.log(
                "Swerve/module " + name + "/drive desired velocity (mps)",
                states.driveDesiredVelocity);
        Telemetry.log(
                "Swerve/module " + name + "/drive velocity (mps)", states.driveVelocity);
        Telemetry.log(
                "Swerve/module " + name + "/drive encoder position (m)", states.driveEncoderPos);
        Telemetry.log(
                "Swerve/module " + name + "/drive voltage (volt)", states.driveVoltage);
        Telemetry.log(
                "Swerve/module " + name + "/turn voltage (volt)", states.turnVoltage);
        Telemetry.log(
                "Swerve/module " + name + "/drive current (amps)", states.driveCurrent);
        Telemetry.log(
                "Swerve/module " + name + "/turn current (amps)", states.turnCurrent);
    }
}
