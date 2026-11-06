package frc.robot;

import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.units.measure.Angle;

import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.gyro.Gyro;

public class CommandFactory {
    private final DriveSubsystem drive;
    private final Gyro gyro;

    public enum TargetZoneType {
        HUB,
        DEPOT_SIDE,
        OUTPOST_SIDE
    }

    public CommandFactory(
            DriveSubsystem drive,
            Gyro gyro) {
        this.drive = drive;
        this.gyro = gyro;
    }

    public Command resetHeading(Angle yaw) {
        return Commands.sequence(
                gyro.setYawCommand(yaw), Commands.runOnce(() -> drive.resetOdometryAfterGyro()));
    }
}
