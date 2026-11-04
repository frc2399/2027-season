package frc.robot.vision;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.SubsystemFactory.RobotType;
import frc.robot.subsystems.drive.DriveConfig;
import java.util.Optional;

public final class VisionPoseEstimator {

    private static Angle CAMERA_PITCH_MAIN; // 0 = horizontal, positive = leaning back
    private static Distance X_ROBOT_TO_CAMERA_OFFSET_MAIN; // positive = in front of robot center
    private static Distance Y_ROBOT_TO_CAMERA_OFFSET_MAIN; // positive = right of robot centerline
    private static Distance Z_ROBOT_TO_CAMERA_OFFSET_MAIN; // ground plane = 0
    private static Angle CAMERA_YAW_MAIN;

    private static Angle CAMERA_PITCH_SECONDARY; // 0 = horizontal, positive = leaning back
    private static Distance
            X_ROBOT_TO_CAMERA_OFFSET_SECONDARY; // positive = in front of robot center
    private static Distance
            Y_ROBOT_TO_CAMERA_OFFSET_SECONDARY; // positive = left of robot centerline
    private static Distance Z_ROBOT_TO_CAMERA_OFFSET_SECONDARY; // ground plane = 0
    private static Angle CAMERA_YAW_SECONDARY;

    private boolean isUsingVisionData = false;

    /** Provides the methods needed to do first-class pose estimation */
    public static interface DriveBase {
        Rotation2d getYaw();

        Rotation2d getYawPerSecond();

        double getLinearSpeed();

        /**
         * Passthrough to {@link edu.wpi.first.math.estimator.SwerveDrivePoseEstimator}
         * addVisionMeasurement
         *
         * @param pose
         * @param timestampSeconds
         * @param visionMeasurementStdDevs
         */
        void addVisionMeasurement(
                Pose2d pose, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs);
    }

    private static Transform3d ROBOT_TO_CAMERA_MAIN;
    private static Transform3d ROBOT_TO_CAMERA_SECONDARY;

    // reject new poses if spinning too fast
    private static final AngularVelocity MAX_ROTATIONS_PER_SECOND = RotationsPerSecond.of(2);
    private static final LinearVelocity MAX_DRIVETRAIN_SPEED_FOR_VISION_UPDATE =
            MetersPerSecond.of(0.8 * DriveConfig.MAX_SPEED.in(MetersPerSecond));

    private final StructPublisher<Pose2d> mt2PublisherMain;
    private final StructPublisher<Pose2d> mt2PublisherSecondary;
    private final DriveBase driveBase;
    private final String limelightNameMain,
            limelightHostnameMain,
            limelightNameSecondary,
            limelightHostnameSecondary;

    /**
     * Create a VisionPoseEstimator
     *
     * @param driveBase the robot drive base to estimate the pose of
     * @param limelightName passed down to calls to LimelightHelpers, useful if you have more than
     *     one Limelight on a robot
     */
    public VisionPoseEstimator(DriveBase driveBase, RobotType robot) {
        this.driveBase = driveBase;
        this.limelightNameMain = CameraConfig.LIMELIGHT_NAME;
        this.limelightHostnameMain =
                "limelight" + (limelightNameMain != "" ? "-" + limelightNameMain : "");
        this.limelightNameSecondary = CameraConfig.SECOND_LIMELIGHT_NAME;
        this.limelightHostnameSecondary =
                "limelight" + (limelightNameSecondary != "" ? "-" + limelightNameSecondary : "");

        mt2PublisherMain =
                NetworkTableInstance.getDefault()
                        .getStructTopic(
                                "VisionPoseEstimator/" + this.limelightNameMain, Pose2d.struct)
                        .publish();
        mt2PublisherMain.setDefault(new Pose2d());

        mt2PublisherSecondary =
                NetworkTableInstance.getDefault()
                        .getStructTopic(
                                "VisionPoseEstimator/" + this.limelightNameSecondary, Pose2d.struct)
                        .publish();
        mt2PublisherSecondary.setDefault(new Pose2d());

        // get values out of config
        CAMERA_PITCH_MAIN = CameraConfig.CAMERA_PITCH;
        CAMERA_YAW_MAIN = CameraConfig.CAMERA_YAW;
        X_ROBOT_TO_CAMERA_OFFSET_MAIN = CameraConfig.X_ROBOT_TO_CAMERA_OFFSET;
        Y_ROBOT_TO_CAMERA_OFFSET_MAIN = CameraConfig.Y_ROBOT_TO_CAMERA_OFFSET;
        Z_ROBOT_TO_CAMERA_OFFSET_MAIN = CameraConfig.Z_ROBOT_TO_CAMERA_OFFSET; // ground plane = 0

        CAMERA_PITCH_SECONDARY = CameraConfig.SECOND_CAMERA_PITCH;
        CAMERA_YAW_SECONDARY = CameraConfig.SECOND_CAMERA_YAW;
        X_ROBOT_TO_CAMERA_OFFSET_SECONDARY = CameraConfig.SECOND_X_ROBOT_TO_CAMERA_OFFSET;
        Y_ROBOT_TO_CAMERA_OFFSET_SECONDARY = CameraConfig.SECOND_Y_ROBOT_TO_CAMERA_OFFSET;
        Z_ROBOT_TO_CAMERA_OFFSET_SECONDARY = CameraConfig.SECOND_Z_ROBOT_TO_CAMERA_OFFSET;

        // meters, radians. Robot origin to camera lens origin
        ROBOT_TO_CAMERA_MAIN =
                new Transform3d(
                        X_ROBOT_TO_CAMERA_OFFSET_MAIN.in(Meters),
                        Y_ROBOT_TO_CAMERA_OFFSET_MAIN.in(Meters),
                        Z_ROBOT_TO_CAMERA_OFFSET_MAIN.in(Meters),
                        new Rotation3d(
                                0, CAMERA_PITCH_MAIN.in(Radians), CAMERA_YAW_MAIN.in(Radians)));

        ROBOT_TO_CAMERA_SECONDARY =
                new Transform3d(
                        X_ROBOT_TO_CAMERA_OFFSET_SECONDARY.in(Meters),
                        Y_ROBOT_TO_CAMERA_OFFSET_SECONDARY.in(Meters),
                        Z_ROBOT_TO_CAMERA_OFFSET_SECONDARY.in(Meters),
                        new Rotation3d(
                                0,
                                CAMERA_PITCH_SECONDARY.in(Radians),
                                CAMERA_YAW_SECONDARY.in(Radians)));

        LimelightHelpers.setCameraPose_RobotSpace(
                limelightNameMain,
                ROBOT_TO_CAMERA_MAIN.getX(),
                ROBOT_TO_CAMERA_MAIN.getY(),
                ROBOT_TO_CAMERA_MAIN.getZ(),
                Math.toDegrees(ROBOT_TO_CAMERA_MAIN.getRotation().getX()),
                Math.toDegrees(ROBOT_TO_CAMERA_MAIN.getRotation().getY()),
                Math.toDegrees(ROBOT_TO_CAMERA_MAIN.getRotation().getZ()));

        LimelightHelpers.setCameraPose_RobotSpace(
                limelightNameSecondary,
                ROBOT_TO_CAMERA_SECONDARY.getX(),
                ROBOT_TO_CAMERA_SECONDARY.getY(),
                ROBOT_TO_CAMERA_SECONDARY.getZ(),
                Math.toDegrees(ROBOT_TO_CAMERA_SECONDARY.getRotation().getX()),
                Math.toDegrees(ROBOT_TO_CAMERA_SECONDARY.getRotation().getY()),
                Math.toDegrees(ROBOT_TO_CAMERA_SECONDARY.getRotation().getZ()));
    }

    /**
     * Get a pose estimate from the configured Limelight, if available
     *
     * @return An Optional containing MegaTag2 pose estimate from the Limelight, or Optional.empty
     *     if it is unavailable or untrustworthy
     */
    public Optional<LimelightHelpers.PoseEstimate> getPoseEstimate(String limelightToEstimate) {
        if (Math.abs(driveBase.getYawPerSecond().getRotations())
                > MAX_ROTATIONS_PER_SECOND.in(RotationsPerSecond)) {
            return Optional.empty();
        } else if (driveBase.getLinearSpeed()
                > MAX_DRIVETRAIN_SPEED_FOR_VISION_UPDATE.in(MetersPerSecond)) {
            return Optional.empty();
        }
        var est =
                Optional.ofNullable(
                        LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightToEstimate));
        if (est.isPresent()) {

            var estimator = est.get();
            // Build a comma-separated string of tag IDs used in the pose estimate
            StringBuilder tagIDs = new StringBuilder();
            if (estimator.rawFiducials != null) {
                for (int i = 0; i < estimator.rawFiducials.length; i++) {
                    if (i > 0) tagIDs.append(", ");
                    tagIDs.append(estimator.rawFiducials[i].id);
                }
            }

            String tagIDString = tagIDs.length() > 0 ? tagIDs.toString() : "None";
            SmartDashboard.putString("vision/poseTagIDs", tagIDString);
        }

        // Reject poses where we can see no tags or are at the "uh oh something went
        // wrong" and reject if either x or y are 0 because then we are in a wall and that's not
        // possible
        return est.filter((pe) -> pe.tagCount > 0 && (pe.pose.getX() != 0 && pe.pose.getY() != 0));
    }

    /** Update the limelight's robot orientation */
    public void periodic() {
        isUsingVisionData = false;
        // Resist the temptation to rotate this depending on alliance - the coordinate
        // system here _has_ to match the WPILib coordinate system, where Yaw is CCW +
        // and 0 faces the red alliance wall
        LimelightHelpers.SetRobotOrientation(
                limelightNameMain, driveBase.getYaw().getDegrees(), 0, 0, 0, 0, 0);
        LimelightHelpers.SetRobotOrientation(
                limelightNameSecondary, driveBase.getYaw().getDegrees(), 0, 0, 0, 0, 0);
        getPoseEstimate(limelightNameMain)
                .ifPresent(
                        (pe) -> {
                            mt2PublisherMain.set(pe.pose);
                            // LimelightHelpers doesn't expose a helper method for these, layout is:
                            // [MT1x, MT1y, MT1z, MT1roll, MT1pitch, MT1Yaw, MT2x, MT2y, MT2z,
                            // MT2roll,
                            // MT2pitch, MT2yaw]
                            var stddevs =
                                    LimelightHelpers.getLimelightNTDoubleArray(
                                            limelightHostnameMain, "stddevs");
                            isUsingVisionData = true;
                            driveBase.addVisionMeasurement(
                                    pe.pose,
                                    pe.timestampSeconds,
                                    VecBuilder.fill(
                                            stddevs[6], stddevs[7], Double.POSITIVE_INFINITY));
                        });
        getPoseEstimate(limelightNameSecondary)
                .ifPresent(
                        (pe) -> {
                            mt2PublisherSecondary.set(pe.pose);
                            // LimelightHelpers doesn't expose a helper method for these, layout is:
                            // [MT1x, MT1y, MT1z, MT1roll, MT1pitch, MT1Yaw, MT2x, MT2y, MT2z,
                            // MT2roll,
                            // MT2pitch, MT2yaw]
                            var stddevs =
                                    LimelightHelpers.getLimelightNTDoubleArray(
                                            limelightHostnameSecondary, "stddevs");
                            driveBase.addVisionMeasurement(
                                    pe.pose,
                                    pe.timestampSeconds,
                                    VecBuilder.fill(
                                            stddevs[6], stddevs[7], Double.POSITIVE_INFINITY));
                        });
        SmartDashboard.putBoolean("vision/is using vision data", isUsingVisionData);
    }
}
