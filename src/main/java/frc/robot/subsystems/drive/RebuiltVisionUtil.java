package frc.robot.subsystems.drive;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Radians;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.Distance;

import frc.robot.CommandFactory.TargetZoneType;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.RobotConstants.TransformConstants;
import frc.robot.util.FieldCalculationHelpers;

import org.wpilib.driverstation.Alliance;
import org.wpilib.telemetry.Telemetry;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class RebuiltVisionUtil {

    private static final double REALIGNING_TOLERANCE_MULTIPLIER = 0.5;

    public static enum ToleranceType {
        REALIGNING,
        POSTALIGNMENT
    }

    public static Supplier<Pose2d> getGoalPose(
            Supplier<Pose2d> robotPose, BooleanSupplier isBlueAlliance) {
        Pose2d returnPose = new Pose2d();
        return () -> returnPose;
    }

    public static Distance getDistanceToAlignmentTarget(Supplier<Pose2d> robotPose) {
        Pose2d targetPose = getAlignmentTargetPose(robotPose);
        if (robotPose.get() == null) {
            return Inches.of(0);
        }
        Pose2d shooterPose =
                robotPose
                        .get()
                        .transformBy(RobotConstants.TransformConstants.ROBOT_TO_SHOOTER_TRANSFORM);
        double distanceBetweenRobotAndHub =
                targetPose.getTranslation().getDistance(shooterPose.getTranslation());
        Telemetry.log(
                "shooter/dist to hub v2 (delete later)", distanceBetweenRobotAndHub);
        return Meters.of(distanceBetweenRobotAndHub);
    }

    public static Pose2d getHubPose() {
        if (FieldConstants.alliance.isPresent() && FieldConstants.alliance.get() == Alliance.Blue) {
            return FieldConstants.BLUE_CENTER_OF_HUB_POSE;
        } else {
            return FieldConstants.RED_CENTER_OF_HUB_POSE;
        }
    }

    public static Pose2d getOutpostSidePose() {
        if (FieldConstants.alliance.isPresent() && FieldConstants.alliance.get() == Alliance.Blue) {
            return FieldConstants.HubConstants.AlignmentTargetPoseConstants.BLUE_OUTPOST_ALIGN_POSE;
        } else {
            return FieldConstants.HubConstants.AlignmentTargetPoseConstants.RED_OUTPOST_ALIGN_POSE;
        }
    }

    public static Pose2d getDepotSidePose() {
        if (FieldConstants.alliance.isPresent() && FieldConstants.alliance.get() == Alliance.Blue) {
            return FieldConstants.HubConstants.AlignmentTargetPoseConstants.BLUE_DEPOT_ALIGN_POSE;
        } else {
            return FieldConstants.HubConstants.AlignmentTargetPoseConstants.RED_DEPOT_ALIGN_POSE;
        }
    }

    public static Pose2d getAlignmentTargetPose(Supplier<Pose2d> robotPose) {
        TargetZoneType targetType = FieldCalculationHelpers.getAlignmentTargetType(robotPose);
        if (targetType == TargetZoneType.HUB) {
            return getHubPose();
        } else if (targetType == TargetZoneType.OUTPOST_SIDE) {
            return getOutpostSidePose();
        } else {
            return getDepotSidePose();
        }
    }

    public static double getDesiredAngleToHub(Supplier<Translation2d> hubToShooterTranslation) {
        if (hubToShooterTranslation.get() == null) {
            return 0;
        }

        Angle desiredAngle =
                Radians.of(
                        Math.atan2(
                                hubToShooterTranslation.get().getY(),
                                hubToShooterTranslation.get().getX()));
        double desiredAngleInDegrees = desiredAngle.in(Degrees);
        return desiredAngleInDegrees;
    }

    public static double getActualAngleToHub(Supplier<Pose2d> robotPose) {
        double actualAngleInDegrees = robotPose.get().getRotation().getDegrees();
        return actualAngleInDegrees;
    }

    // public double getRangeOfAngleToHubInDegrees(
    //         double robotDistanceToHub, Pose2d orientTargetPose, Pose2d poseToOrientToTarget) {
    //     double hubRadiusMinusFuelRadiusInMeters =
    //             HubConstants.HUB_RADIUS.minus(HubConstants.FUEL_RADIUS).in(Meters);

    //     Translation2d targetToRobotTranslation =
    //             orientTargetPose.getTranslation().minus(poseToOrientToTarget.getTranslation());
    //     double hubToRobotDistance = targetToRobotTranslation.getNorm();
    //     double rangeOfAngleToHubInDegrees =
    //             Math.atan2(hubRadiusMinusFuelRadiusInMeters, hubToRobotDistance);
    //     return rangeOfAngleToHubInDegrees;
    // }

    // abstracted because the target pose could either be the CENTER of the hub or one of the SIDES
    // of the hub
    public static Translation2d getTargetToShooterTranslation(
            Supplier<Pose2d> shooterPose, Supplier<Pose2d> targetPose) {
        return targetPose.get().getTranslation().minus(shooterPose.get().getTranslation());
    }

    public static double getMaxDiffOfDesiredAndActualAngleInDegrees(
            Translation2d hubToShooterTranslation, Pose2d shooterPose) {
        double hubRadiusMinusFuelRadiusInMeters =
                FieldConstants.HubConstants.AlignmentTargetPoseConstants.HUB_RADIUS
                        .minus(FieldConstants.HubConstants.AlignmentTargetPoseConstants.FUEL_RADIUS)
                        .in(Meters);

        Transform2d toEdgeOfHubTransform =
                new Transform2d(0, hubRadiusMinusFuelRadiusInMeters, Rotation2d.kZero);
        Pose2d edgeOfHubPose = getHubPose().transformBy(toEdgeOfHubTransform);

        Translation2d edgeOfHubToShooterTranslation =
                getTargetToShooterTranslation(() -> shooterPose, () -> edgeOfHubPose);

        double dotProduct = hubToShooterTranslation.dot(edgeOfHubToShooterTranslation);
        double productOfMagnitudes =
                hubToShooterTranslation.getNorm() * edgeOfHubToShooterTranslation.getNorm();

        Angle maxDiffOfAngles = Radians.of(Math.acos(dotProduct / productOfMagnitudes));
        return maxDiffOfAngles.in(Degrees);
    }

    public static boolean isShootingAngleAlignedToHub(
            Supplier<Pose2d> robotPose, ToleranceType toleranceType) {
        Pose2d shooterPose =
                robotPose.get().transformBy(TransformConstants.ROBOT_TO_SHOOTER_TRANSFORM);
        Translation2d hubToShooterTranslation =
                getTargetToShooterTranslation(() -> shooterPose, () -> getHubPose());

        double robotAngleToHubInDegrees = getDesiredAngleToHub(() -> hubToShooterTranslation);
        Telemetry.log("vision/util/des angle", robotAngleToHubInDegrees);

        double diffOfAngleInDegrees = getActualAngleToHub(robotPose) - robotAngleToHubInDegrees;
        Telemetry.log("vision/util/diff angle", diffOfAngleInDegrees);

        double maxDiffOfAnglesInDegrees =
                getMaxDiffOfDesiredAndActualAngleInDegrees(hubToShooterTranslation, shooterPose);
        Telemetry.log("vision/util/max diff", maxDiffOfAnglesInDegrees);

        if (toleranceType == ToleranceType.REALIGNING) {
            maxDiffOfAnglesInDegrees *= REALIGNING_TOLERANCE_MULTIPLIER;
        }

        return Math.abs(diffOfAngleInDegrees) <= Math.abs(maxDiffOfAnglesInDegrees);
    }
}
