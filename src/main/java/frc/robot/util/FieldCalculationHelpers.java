package frc.robot.util;

import static org.wpilib.units.Units.Meters;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.driverstation.DriverStation
import org.wpilib.driverstation.Alliance;
import frc.robot.CommandFactory.TargetZoneType;
import frc.robot.constants.FieldConstants;
import java.util.function.Supplier;

public class FieldCalculationHelpers {
    public static Boolean shouldTargetHub(Supplier<Pose2d> robotLocation) {
        double poseX = robotLocation.get().getX();
        if (FieldConstants.alliance.isPresent()) {
            if (FieldConstants.alliance.get() == DriverStation.Alliance.Blue) {
                return (FieldConstants.FieldBoundaries.BLUE_ZONE_BOUNDARY_X.in(Meters) >= poseX);
            }
            if (FieldConstants.alliance.get() == DriverStation.Alliance.Red) {
                return (FieldConstants.FieldBoundaries.RED_ZONE_BOUNDARY_X.in(Meters) <= poseX);
            }
        }
        return false;
    }

    public static boolean amInDangerZone(Supplier<Pose2d> robotLocation) {
        if (shouldTargetHub(robotLocation)) return false;
        double y = robotLocation.get().getY();
        double yScreenLine = FieldConstants.FieldBoundaries.HORIZONTAL_CENTER_LINE.in(Meters);
        if (Math.abs(y - yScreenLine) < FieldConstants.FieldBoundaries.SCREEN_RANGE.in(Meters))
            return true;
        return false;
    }

    public static TargetZoneType getAlignmentTargetType(Supplier<Pose2d> robotLocation) {
        double poseY = robotLocation.get().getY();
        if (shouldTargetHub(robotLocation)) {
            return TargetZoneType.HUB;
        }
        if (FieldConstants.alliance.isPresent()
                && FieldConstants.alliance.get() == DriverStation.Alliance.Blue) {
            if (FieldConstants.FieldBoundaries.HORIZONTAL_CENTER_LINE.in(Meters) <= poseY) {

                return TargetZoneType.DEPOT_SIDE;
            } else {

                return TargetZoneType.OUTPOST_SIDE;
            }
        } else {
            if (FieldConstants.FieldBoundaries.HORIZONTAL_CENTER_LINE.in(Meters) >= poseY) {

                return TargetZoneType.DEPOT_SIDE;
            } else {

                return TargetZoneType.OUTPOST_SIDE;
            }
        }
    }
}
