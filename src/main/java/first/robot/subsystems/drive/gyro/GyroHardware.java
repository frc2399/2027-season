// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package first.robot.subsystems.drive.gyro;

import static org.wpilib.units.Units.Degrees;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;
import org.wpilib.units.measure.Angle;
import first.robot.constants.RobotConstants;

/** IO implementation for Pigeon2 */
public class GyroHardware implements GyroIO {
    private final Pigeon2 pigeon;
    private final CANBus canBus;

    public GyroHardware() {
        canBus = new CANBus("rio");
        pigeon = new Pigeon2(RobotConstants.MotorIdConstants.GYRO_CAN_ID, canBus);
        // these three lines disable all status signal readings off the pigeon except
        // for the three we need (the three directly below). this reduces CAN network
        // utilization
        pigeon.getYaw(false)
                .setUpdateFrequency(RobotConstants.SpeedConstants.MAIN_LOOP_FREQUENCY_HZ);
        pigeon.getAngularVelocityZDevice()
                .setUpdateFrequency(RobotConstants.SpeedConstants.MAIN_LOOP_FREQUENCY_HZ);
        pigeon.getFault_Hardware()
                .setUpdateFrequency(RobotConstants.SpeedConstants.MAIN_LOOP_FREQUENCY_HZ);
        pigeon.optimizeBusUtilization();
    }

    public Angle getYaw(boolean refresh) {
        return Degrees.of(pigeon.getYaw(refresh).getValueAsDouble());
    }

    public void setYaw(Angle yaw) {
        pigeon.setYaw(yaw.in(Degrees));
    }

    public StatusSignal<org.wpilib.units.measure.AngularVelocity> getAngularVelocity() {
        // Don't refresh the status signal by default, we already get it at
        // MAIN_LOOP_FREQUENCY_HZ, and refreshing blocks, causing loop overruns
        return this.getAngularVelocity(false);
    }

    public StatusSignal<org.wpilib.units.measure.AngularVelocity> getAngularVelocity(
            boolean refresh) {
        return pigeon.getAngularVelocityZDevice(refresh);
    }

    public boolean hasFault() {
        // Don't refresh the status signal by default, we already get it at
        // MAIN_LOOP_FREQUENCY_HZ, and refreshing blocks, causing loop overruns
        return this.hasFault(false);
    }

    public boolean hasFault(boolean refresh) {
        return pigeon.getFault_Hardware(refresh).getValue();
    }
}
