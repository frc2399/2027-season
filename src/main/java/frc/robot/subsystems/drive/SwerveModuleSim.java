package frc.robot.subsystems.drive;

import org.wpilib.math.system.DCMotor;
import org.wpilib.math.system.LinearSystem;
import org.wpilib.math.system.Models;
import org.wpilib.math.util.MathUtil;
import org.wpilib.math.util.Units;
import org.wpilib.simulation.DCMotorSim;
import org.wpilib.telemetry.Telemetry;

/**
 * Simulation implementation of the SwerveModuleIO interface using WPILib's DCMotorSim. This
 * provides a basic physics simulation for testing swerve drive control logic without requiring
 * physical hardware.
 */
public class SwerveModuleSim implements SwerveModuleIO {
    private static final double LOOP_PERIOD_SECS = 0.02;

    // Physical constants for SDS MK4i L2 modules with NEO Vortex drive and NEO 550 turn
    // Drive motor: NEO Vortex through a reduction of (45 * 22) / (12 * 15) = 5.5:1
    private static final double DRIVING_MOTOR_REDUCTION = (45.0 * 22.0) / (12.0 * 15.0);

    // Turn motor: NEO 550 with Through Bore Encoder (1:1 gearing on the output shaft)
    private static final double TURNING_MOTOR_REDUCTION = 1.0;

    // Wheel parameters
    private static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(3.0);

    // Moments of inertia (kg*m^2)
    // Drive MOI includes wheel mass and rotating parts
    // Using a much smaller value for faster acceleration in sim
    private static final double DRIVE_MOI = 0.003;
    // Turn MOI is for the steering mechanism
    private static final double TURN_MOI = 0.001;

    // Motor simulations
    private final DCMotorSim driveSim;
    private final DCMotorSim turnSim;

    // Simulated absolute encoder offset (random starting position)
    private final double turnAbsoluteInitPosition;

    // Applied voltages (clamped to battery limits)
    private double driveAppliedVolts = 0.0;
    private double turnAppliedVolts = 0.0;

    // Desired setpoints for telemetry
    private double desiredDriveSpeedMPS = 0.0;
    private double desiredTurnAngle = 0.0;

    private final double chassisAngularOffset;
    private final String name;

    /**
     * Creates a new simulated swerve module.
     *
     * @param chassisAngularOffset The angular offset of the module from the chassis
     * @param name The name of the module for telemetry (e.g., "front left")
     */
    public SwerveModuleSim(double chassisAngularOffset, String name) {
        this.chassisAngularOffset = chassisAngularOffset;
        this.name = name;

        // Create drive motor simulation
        // NEO Vortex motor with gear reduction and moment of inertia
        DCMotor driveGearbox = DCMotor.getNeoVortex(1);
        driveSim =
                // new DCMotorSim(
                //         LinearSystemId.createDCMotorSystem(
                //                 driveGearbox, DRIVE_MOI, DRIVING_MOTOR_REDUCTION),
                //         driveGearbox);

        // Create turn motor simulation
        // NEO 550 motor with gear reduction and moment of inertia
        // DCMotor turnGearbox = DCMotor.getNeo550(1);
        // turnSim =
        //         new DCMotorSim(
        //                 Models.singleJointedArmFromPhysicalConstants(
        //                         turnGearbox, TURN_MOI, TURNING_MOTOR_REDUCTION),
        //                 turnGearbox);

        // Set initial turn position to account for chassis angular offset
        // Add some randomness to simulate real encoder positions
        turnAbsoluteInitPosition = chassisAngularOffset + (Math.random() * 0.1 - 0.05);
    }

    @Override
    public void setDriveEncoderPosition(double position) {
        // Convert from meters to radians at the OUTPUT shaft
        // DCMotorSim works with output shaft position (already accounting for gearing)
        double outputRadians = position / (WHEEL_DIAMETER_METERS / 2.0);
        driveSim.setState(outputRadians, driveSim.getAngularVelocity());
    }

    @Override
    public double getDriveEncoderPosition() {
        // DCMotorSim.getAngularPositionRad() returns OUTPUT shaft position
        // (already accounts for gear reduction)
        double outputRadians = driveSim.getAngularPosition();
        return outputRadians * (WHEEL_DIAMETER_METERS / 2.0);
    }

    @Override
    public void setDesiredDriveSpeedMPS(double speed, boolean isFlipped) {
        this.desiredDriveSpeedMPS = speed;
        // Simple voltage control based on desired speed
        // In real hardware, closed-loop control runs on the SparkFlex
        // For sim, we use a simple proportional controller with feedforward

        double currentSpeedMPS = getDriveEncoderSpeedMPS();
        double error = speed - currentSpeedMPS;

        // Calculate theoretical max speed for NEO Vortex:
        // Free speed: 6784 RPM, Gear ratio: 5.5:1, Wheel diameter: 3 inches (0.0762m)
        // Wheel circumference = π * 0.0762 = 0.2394m
        // Max wheel speed = 6784 / 5.5 = 1233.45 RPM = 20.56 RPS
        // Max linear speed = 20.56 * 0.2394 = 4.92 m/s
        final double MAX_SPEED_MPS = 4.92;

        // Feedforward: voltage proportional to desired speed
        // Use 11V at max speed (not 12V to account for losses)
        double feedforward = (speed / MAX_SPEED_MPS) * 11.0;

        // Proportional gain for error correction
        double proportional = error * 3.0; // Increased P gain for faster response

        driveAppliedVolts = Math.clamp(feedforward + proportional, -12.0, 12.0);
    }

    @Override
    public double getDriveEncoderSpeedMPS() {
        // DCMotorSim.getAngularVelocityRadPerSec() returns OUTPUT shaft velocity
        // (already accounts for gear reduction), so no need to divide again
        double outputRadPerSec = driveSim.getAngularVelocity();
        double speedMPS = outputRadPerSec * (WHEEL_DIAMETER_METERS / 2.0);

        // Debug output for troubleshooting
        Telemetry.log(
                "Swerve/module " + name + "/sim output rad per sec", outputRadPerSec);
        Telemetry.log(
                "Swerve/module " + name + "/sim wheel radius", WHEEL_DIAMETER_METERS / 2.0);
        Telemetry.log(
                "Swerve/module " + name + "/sim drive reduction", DRIVING_MOTOR_REDUCTION);

        return speedMPS;
    }

    @Override
    public double getTurnEncoderPosition() {
        // Return absolute position including random initial offset
        double relativePosition = turnSim.getAngularPosition() / TURNING_MOTOR_REDUCTION;
        return MathUtil.angleModulus(relativePosition + turnAbsoluteInitPosition);
    }

    @Override
    public void setDesiredTurnAngle(double angle) {
        this.desiredTurnAngle = angle;

        // Simple position control
        // In real hardware, closed-loop control runs on the SparkMax
        double currentAngle = turnSim.getAngularPosition() / TURNING_MOTOR_REDUCTION;
        double targetAngle = angle - turnAbsoluteInitPosition;

        // Calculate shortest path to target
        double error = MathUtil.angleModulus(targetAngle - currentAngle);

        // PD controller
        // double proportional = error * 10.0; // P gain
        double proportional = error * 2.0; // P gain
        double derivative = -turnSim.getAngularVelocity() * 0.1; // D gain

        turnAppliedVolts = Math.clamp(proportional + derivative, -12.0, 12.0);
    }

    @Override
    public double getChassisAngularOffset() {
        return chassisAngularOffset;
    }

    @Override
    public void updateStates(SwerveModuleIOStates states) {
        // Update physics simulations
        driveSim.setInputVoltage(driveAppliedVolts);
        driveSim.update(LOOP_PERIOD_SECS);

        turnSim.setInputVoltage(turnAppliedVolts);
        turnSim.update(LOOP_PERIOD_SECS);

        // Populate states for telemetry
        states.driveEncoderPos = getDriveEncoderPosition();
        states.driveVelocity = getDriveEncoderSpeedMPS();
        states.driveDesiredVelocity = desiredDriveSpeedMPS;
        states.driveVoltage = driveAppliedVolts;
        states.driveCurrent = Math.abs(driveSim.getCurrentDraw());

        states.turnAngle = Units.radiansToDegrees(MathUtil.angleModulus(getTurnEncoderPosition()));
        states.desiredAngle = Units.radiansToDegrees(MathUtil.angleModulus(desiredTurnAngle));
        states.turnVoltage = turnAppliedVolts;
        states.turnCurrent = Math.abs(turnSim.getCurrentDraw());
        states.turningEncoderPos = getTurnEncoderPosition();

        // Debug: Log chassis offset and initial position
        Telemetry.log(
                "Swerve/module " + name + "/chassis offset(deg)",
                Units.radiansToDegrees(chassisAngularOffset));
        Telemetry.log(
                "Swerve/module " + name + "/init position(deg)",
                Units.radiansToDegrees(turnAbsoluteInitPosition));

        // Log to SmartDashboard for debugging
        Telemetry.log(
                "Swerve/module " + name + "/turn desired angle(deg)", states.desiredAngle);
        Telemetry.log("Swerve/module " + name + "/turn angle(deg)", states.turnAngle);
        Telemetry.log(
                "Swerve/module " + name + "/drive desired velocity(mps)",
                states.driveDesiredVelocity);
        Telemetry.log(
                "Swerve/module " + name + "/drive velocity(mps)", states.driveVelocity);
        Telemetry.log(
                "Swerve/module " + name + "/drive encoder position(m)", states.driveEncoderPos);
        Telemetry.log(
                "Swerve/module " + name + "/drive voltage(volt)", states.driveVoltage);
        Telemetry.log(
                "Swerve/module " + name + "/turn voltage(volt)", states.turnVoltage);
        Telemetry.log(
                "Swerve/module " + name + "/drive current(amps)", states.driveCurrent);
        Telemetry.log(
                "Swerve/module " + name + "/turn current(amps)", states.turnCurrent);
    }
}
