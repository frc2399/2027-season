package first.robot.subsystems.drive.gyro;

import static org.wpilib.units.Units.Radians;

import com.ctre.phoenix6.StatusSignal;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;

/**
 * Simulated gyro implementation that integrates angular velocity to calculate heading. This allows
 * the robot to properly rotate in simulation.
 *
 * <p>Usage: The DriveSubsystem should call update() in simulationPeriodic() and pass the chassis
 * angular velocity from kinematics.
 */
public class GyroSim implements GyroIO {
    private double yawRadians = 0.0;
    private double angularVelocityRadPerSec = 0.0;

    /**
     * Sets the simulated angular velocity. This should be called from the drive subsystem based on
     * the chassis angular velocity calculated from swerve kinematics.
     *
     * @param omegaRadPerSec The angular velocity in radians per second
     */
    public void setAngularVelocity(double omegaRadPerSec) {
        this.angularVelocityRadPerSec = omegaRadPerSec;
    }

    /**
     * Updates the simulated gyro by integrating angular velocity. This should be called
     * periodically (typically every 20ms).
     */
    public void update(double dtSeconds) {
        // Integrate angular velocity to get heading
        yawRadians += angularVelocityRadPerSec * dtSeconds;

        // Normalize angle to [-pi, pi]
        yawRadians = new Rotation2d(yawRadians).getRadians();
    }

    @Override
    public Angle getYaw(boolean refresh) {
        return Radians.of(yawRadians);
    }

    @Override
    public void setYaw(Angle yaw) {
        this.yawRadians = yaw.in(Radians);
    }

    @Override
    public StatusSignal<AngularVelocity> getAngularVelocity() {
        // For simulation, return a simple StatusSignal
        StatusSignal<AngularVelocity> signal = new StatusSignal<>(null, null, null);

        return signal;
    }

    /**
     * Gets the angular velocity in radians per second. Use this instead of
     * getAngularVelocity().getValueAsDouble() in simulation.
     */
    public double getAngularVelocityRadPerSec() {
        return angularVelocityRadPerSec;
    }

    @Override
    public boolean hasFault() {
        return false; // No faults in simulation
    }
}
