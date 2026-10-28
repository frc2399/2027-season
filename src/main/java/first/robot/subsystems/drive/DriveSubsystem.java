// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot.subsystems.drive;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.Radians;
import static org.wpilib.units.Units.RadiansPerSecond;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.FlippingUtil;
import com.pathplanner.lib.util.PathPlannerLogging;
import org.wpilib.math.Matrix;
import org.wpilib.math.controller.PIDController;
import org.wpilib.math.estimator.SwerveDrivePoseEstimator;
import org.wpilib.math.filter.Debouncer;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.kinematics.SwerveDriveKinematics;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.kinematics.SwerveModuleVelocity;
import org.wpilib.math.numbers.N1;
import org.wpilib.math.numbers.N3;
import org.wpilib.math.util.Units;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.networktables.StructArrayPublisher;
import org.wpilib.networktables.StructPublisher;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.LinearVelocity;
import org.wpilib.util.Alert;
import org.wpilib.wpilibj.Alert.AlertType;
import org.wpilib.wpilibj.DriverStation;
import org.wpilib.wpilibj.DriverStation.Alliance;
import org.wpilib.wpilibj.Timer;
import org.wpilib.wpilibj.smartdashboard.Field2d;
import org.wpilib.wpilibj.smartdashboard.FieldObject2d;
import org.wpilib.telemetry.Telemetry;
import org.wpilib.wpilibj2.command.Command;
import org.wpilib.wpilibj2.command.SubsystemBase;
import first.robot.Robot;
import first.robot.constants.FieldConstants;
import first.robot.constants.RobotConstants;
import first.robot.constants.RobotConstants.TransformConstants;
import first.robot.subsystems.drive.gyro.Gyro;
import first.robot.util.FieldCalculationHelpers;
import first.robot.util.GameState;
import first.robot.vision.VisionPoseEstimator.DriveBase;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class DriveSubsystem extends SubsystemBase implements DriveBase {
    // for drivetopose
    private boolean atGoal = true;
    private BooleanSupplier isBlueAlliance;
    private boolean isAutoOrienting = false;

    private DriveSubsystemStates states = new DriveSubsystemStates();

    // correction PID
    private double HEADING_P;
    private double HEADING_D;

    private PIDController drivePIDController;

    private RobotConfig config;

    // debouncer for turning
    private double ROTATION_DEBOUNCE_TIME = 0.5;
    private Debouncer rotationDebouncer = new Debouncer(ROTATION_DEBOUNCE_TIME);

    private final double MIN_STRAFE_SPEED = 0.075;

    // Odometry
    private SwerveDrivePoseEstimator poseEstimator;
    private Pose2d robotPose;

    // swerve modules
    private SwerveModule frontLeft;
    private SwerveModule frontRight;
    private SwerveModule rearLeft;
    private SwerveModule rearRight;

    private final Distance TRACK_WIDTH;
    private final Distance WHEEL_BASE;
    private final LinearVelocity MAX_LINEAR_SPEED;
    private final AngularVelocity MAX_ANGULAR_VELOCITY;

    public final Translation2d FRONT_LEFT_OFFSET;
    public final Translation2d REAR_LEFT_OFFSET;
    public final Translation2d FRONT_RIGHT_OFFSET;
    public final Translation2d REAR_RIGHT_OFFSET;

    private final SwerveDriveKinematics DRIVE_KINEMATICS;

    private static final double TRANSLATION_P = 5.0;
    private static final double TRANSLATION_I = 0.0;
    private static final double TRANSLATION_D = 0.0;

    private static final double ROTATION_P = 5.0;
    private static final double ROTATION_I = 0.0;
    private static final double ROTATION_D = 0.0;

    // Slew rate filter variables for controlling lateral acceleration
    private double desiredAngle = 0;
    private Gyro gyro;

    Alert alert = new Alert("Fault detected on pigeon", AlertType.kError);

    private final Field2d field2d = new Field2d();
    private FieldObject2d frontLeftField2dModule = field2d.getObject("front left module");
    private FieldObject2d rearLeftField2dModule = field2d.getObject("rear left module");
    private FieldObject2d frontRightField2dModule = field2d.getObject("front right module");
    private FieldObject2d rearRightField2dModule = field2d.getObject("rear right module");

    private ChassisVelocities relativeRobotSpeeds = new ChassisVelocities();

    private Rotation2d lastAngle = new Rotation2d();

    // public static final RobotConfig CONFIG = new RobotConfig(ROBOT_MASS,
    // ROBOT_MOI, new
    // ModuleConfig(
    // WHEEL_RADIUS, MAX_SPEED, WHEEL_COF, DRIVE_MOTOR, DRIVE_CURRENT_LIMIT,
    // NUM_MOTORS),
    // MODULE_OFFSETS);

    public static class DriveSubsystemStates {
        public Pose2d pose = new Pose2d();
        public double poseTheta = 0;
        public double velocityXMPS = 0;
        public double velocityYMPS = 0;
        public double totalVelocity = 0;
        public double gyroAngleDegrees = 0;
        public double angularVelocity = 0;
        public double driveEncoderPos = 0.0;
    }

    StructArrayPublisher<SwerveModuleVelocity> SwerveModuleVelocityPublisher =
            NetworkTableInstance.getDefault()
                    .getStructArrayTopic(
                            "/SmartDashboard/drive/Current Modules States",
                            SwerveModuleVelocity.struct)
                    .publish();

    StructArrayPublisher<SwerveModuleVelocity> swerveModuleDesiredStatePublisher =
            NetworkTableInstance.getDefault()
                    .getStructArrayTopic(
                            "/SmartDashboard/drive/Desired Modules States",
                            SwerveModuleVelocity.struct)
                    .publish();

    // Useful pose debugging
    private final StructPublisher<Pose2d> posePublisher;

    /** Creates a new DriveSubsystem. */
    public DriveSubsystem(
            SwerveModule frontLeft,
            SwerveModule frontRight,
            SwerveModule rearLeft,
            SwerveModule rearRight,
            Gyro gyro) {
        this.gyro = gyro;
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.rearLeft = rearLeft;
        this.rearRight = rearRight;

        // get from config!
        HEADING_P = DriveConfig.HEADING_P;
        HEADING_D = DriveConfig.HEADING_D;
        MAX_LINEAR_SPEED = DriveConfig.MAX_SPEED;
        MAX_ANGULAR_VELOCITY = DriveConfig.MAX_ANGULAR_VELOCITY;
        TRACK_WIDTH = DriveConfig.TRACK_WIDTH;
        WHEEL_BASE = DriveConfig.TRACK_LENGTH;

        drivePIDController = new PIDController(HEADING_P, 0, HEADING_D);

        Telemetry.log("drive/config/heading_p", HEADING_P);
        Telemetry.log("drive/config/heading_d", HEADING_D);
        Telemetry.log(
                "drive/config/max_linear_speed", MAX_LINEAR_SPEED.in(MetersPerSecond));
        Telemetry.log(
                "drive/config/max_angular_vel", MAX_ANGULAR_VELOCITY.in(RadiansPerSecond));
        Telemetry.log("drive/config/trackwidth", TRACK_WIDTH.in(Inches));
        Telemetry.log("drive/config/tracklength", WHEEL_BASE.in(Inches));

        FRONT_LEFT_OFFSET =
                new Translation2d(WHEEL_BASE.in(Meters) / 2, TRACK_WIDTH.in(Meters) / 2);
        REAR_LEFT_OFFSET =
                new Translation2d(-WHEEL_BASE.in(Meters) / 2, TRACK_WIDTH.in(Meters) / 2);
        FRONT_RIGHT_OFFSET =
                new Translation2d(WHEEL_BASE.in(Meters) / 2, -TRACK_WIDTH.in(Meters) / 2);
        REAR_RIGHT_OFFSET =
                new Translation2d(-WHEEL_BASE.in(Meters) / 2, -TRACK_WIDTH.in(Meters) / 2);

        DRIVE_KINEMATICS =
                new SwerveDriveKinematics(
                        FRONT_LEFT_OFFSET, FRONT_RIGHT_OFFSET, REAR_LEFT_OFFSET, REAR_RIGHT_OFFSET);

        SmartDashboard.putData(field2d);
        SmartDashboard.putData("drive/Drivetrain Commands", this);
        poseEstimator =
                new SwerveDrivePoseEstimator(
                        DRIVE_KINEMATICS,
                        Rotation2d.fromDegrees(gyro.getYaw(false).in(Degrees)),
                        new SwerveModulePosition[] {
                            frontLeft.getPosition(),
                            frontRight.getPosition(),
                            rearLeft.getPosition(),
                            rearRight.getPosition()
                        },
                        new Pose2d(0, 0, new Rotation2d(gyro.getYaw(false))));
        // file rather than
        // free-floating numbers
        posePublisher =
                NetworkTableInstance.getDefault()
                        .getStructTopic("drive/EstimatedPose", Pose2d.struct)
                        .publish();
        posePublisher.setDefault(new Pose2d());

        try {
            config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                    this::getPose,
                    this::resetOdometry,
                    this::getRobotRelativeSpeeds,
                    (speeds, feedforwards) -> setRobotRelativeSpeeds(speeds),
                    new PPHolonomicDriveController(
                            new PIDConstants(TRANSLATION_P, TRANSLATION_I, TRANSLATION_D),
                            new PIDConstants(ROTATION_P, ROTATION_I, ROTATION_D)),
                    config, // The robot configuration
                    () -> {
                        if (FieldConstants.alliance.isPresent()) {
                            return FieldConstants.alliance.get() == DriverStation.Alliance.Red;
                        }
                        return false;
                    },
                    this);
        } catch (Exception e) {
            DriverStation.reportError(
                    "Failed to load Pathplanner config and configure Autobuilder",
                    e.getStackTrace());
        }
        configurePathPlannerLogging();
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("/drive/atGoal", atGoal);
        SmartDashboard.putBoolean("/drive/isHubActive", GameState.isHubActive(0));
        // This will get the simulated sensor readings that we set
        // in the previous article while in simulation, but will use
        // real values on the robot itself.
        poseEstimator.updateWithTime(
                Timer.getFPGATimestamp(),
                Rotation2d.fromRadians(gyro.getYaw(true).in(Radians)),
                new SwerveModulePosition[] {
                    frontLeft.getPosition(),
                    frontRight.getPosition(),
                    rearLeft.getPosition(),
                    rearRight.getPosition()
                });

        robotPose = getPose();
        field2d.setRobotPose(robotPose);

        SmartDashboard.putBoolean(
                "robot/should Target Hub",
                FieldCalculationHelpers.shouldTargetHub(() -> robotPose));
        SmartDashboard.putBoolean(
                "robot/am in Danger Zone", FieldCalculationHelpers.amInDangerZone(() -> robotPose));
        SmartDashboard.putBoolean(
                "robot/isShootingAngleAlignedToHub wide tolerance",
                RebuiltVisionUtil.isShootingAngleAlignedToHub(
                        () -> robotPose, RebuiltVisionUtil.ToleranceType.POSTALIGNMENT));
        SmartDashboard.putBoolean(
                "robot/isShootingAngleAlignedToHub narrow tolerance",
                RebuiltVisionUtil.isShootingAngleAlignedToHub(
                        () -> robotPose, RebuiltVisionUtil.ToleranceType.REALIGNING));
        SmartDashboard.putString(
                "robot/where should robot pass",
                FieldCalculationHelpers.getAlignmentTargetType(() -> robotPose).toString());

        frontLeftField2dModule.setPose(
                robotPose.transformBy(
                        new Transform2d(
                                FRONT_LEFT_OFFSET,
                                new Rotation2d(frontLeft.getTurnEncoderPosition()))));

        rearLeftField2dModule.setPose(
                robotPose.transformBy(
                        new Transform2d(
                                REAR_LEFT_OFFSET,
                                new Rotation2d(rearLeft.getTurnEncoderPosition()))));

        frontRightField2dModule.setPose(
                robotPose.transformBy(
                        new Transform2d(
                                FRONT_RIGHT_OFFSET,
                                new Rotation2d(frontRight.getTurnEncoderPosition()))));

        rearRightField2dModule.setPose(
                robotPose.transformBy(
                        new Transform2d(
                                REAR_RIGHT_OFFSET,
                                new Rotation2d(rearRight.getTurnEncoderPosition()))));

        SwerveModuleVelocity[] SwerveModuleVelocitys =
                new SwerveModuleVelocity[] {
                    frontLeft.getState(),
                    frontRight.getState(),
                    rearLeft.getState(),
                    rearRight.getState(),
                };
        SwerveModuleVelocityPublisher.set(SwerveModuleVelocitys);

        if (Robot.isSimulation()) {
            double angleChange =
                    DRIVE_KINEMATICS.toChassisVelocities(SwerveModuleVelocitys).omegaRadiansPerSecond
                            * (1 / RobotConstants.SpeedConstants.MAIN_LOOP_FREQUENCY_HZ);
            lastAngle = lastAngle.plus(Rotation2d.fromRadians(angleChange));
            gyro.setYawCommand(Radians.of(lastAngle.getRadians()));
        }

        logAndUpdateDriveSubsystemStates();
        alert.set(gyro.hasFault());

        frontLeft.updateStates();
        frontRight.updateStates();
        rearLeft.updateStates();
        rearRight.updateStates();
    }

    /** Returns the currently-estimated pose of the robot. */
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /** Returns the current odometry rotation. */
    public Rotation2d getRotation() {
        return getPose().getRotation();
    }

    /** Resets the odometry to the specified pose. */
    public void resetOdometry(Pose2d pose) {
        poseEstimator.resetPosition(
                Rotation2d.fromRadians(gyro.getYaw(false).in(Radians)),
                new SwerveModulePosition[] {
                    frontLeft.getPosition(),
                    frontRight.getPosition(),
                    rearLeft.getPosition(),
                    rearRight.getPosition()
                },
                pose);
    }

    // basically, when we reset gyro, m_gyroOffset in SwerveDrivePoseEstimator does
    // NOT reset so the pose does not reset properly :(
    public void resetOdometryAfterGyro() {
        poseEstimator.resetRotation(Rotation2d.fromRadians(gyro.getYaw(true).in(Radians)));
    }

    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed Speed of the robot in the x direction (forward).
     * @param ySpeed Speed of the robot in the y direction (sideways).
     * @param rotRate Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to the field.
     */
    public Command driveCommand(
            DoubleSupplier xSpeed,
            DoubleSupplier ySpeed,
            DoubleSupplier rotRate,
            Boolean fieldRelative,
            BooleanSupplier shouldAutoOrient) {
        return this.run(
                        () -> {
                            double currentAngle = gyro.getYaw(false).in(Radians);
                            if (FieldConstants.alliance.isPresent()
                                    && FieldConstants.alliance.get()
                                            == DriverStation.Alliance.Red) {
                                currentAngle += Math.PI;
                            }

                            isAutoOrienting = shouldAutoOrient.getAsBoolean();

                            double r = Math.hypot(xSpeed.getAsDouble(), ySpeed.getAsDouble());
                            double polarAngle =
                                    Math.atan2(ySpeed.getAsDouble(), xSpeed.getAsDouble());
                            double polarXSpeed = r * Math.cos(polarAngle);
                            double polarYSpeed = r * Math.sin(polarAngle);

                            // //Account for edge case when gyro resets
                            if (currentAngle == 0) {
                                desiredAngle = 0;
                            }

                            double newRotRate =
                                    getRotRate(
                                            currentAngle,
                                            Math.pow(rotRate.getAsDouble(), 3),
                                            polarXSpeed,
                                            polarYSpeed,
                                            shouldAutoOrient);

                            // Convert the commanded speeds into the correct units for the
                            // drivetrain
                            double xSpeedDelivered =
                                    polarXSpeed * MAX_LINEAR_SPEED.in(MetersPerSecond);
                            double ySpeedDelivered =
                                    polarYSpeed * MAX_LINEAR_SPEED.in(MetersPerSecond);
                            double rotRateDelivered =
                                    newRotRate * MAX_ANGULAR_VELOCITY.in(RadiansPerSecond);

                            if (fieldRelative) {
                                relativeRobotSpeeds =
                                        ChassisVelocities.fromFieldRelativeSpeeds(
                                                xSpeedDelivered,
                                                ySpeedDelivered,
                                                rotRateDelivered,
                                                Rotation2d.fromRadians(currentAngle));
                            } else {
                                relativeRobotSpeeds =
                                        new ChassisVelocities(
                                                xSpeedDelivered, ySpeedDelivered, rotRateDelivered);
                            }

                            Telemetry.log("drive/xSpeedDelivered", xSpeedDelivered);
                            Telemetry.log("drive/ySpeedDelivered", ySpeedDelivered);
                            Telemetry.log("drive/PolarAngle", polarAngle);

                            var SwerveModuleVelocitys =
                                    DRIVE_KINEMATICS.toSwerveModuleVelocitys(relativeRobotSpeeds);
                            SwerveDriveKinematics.desaturateWheelSpeeds(
                                    SwerveModuleVelocitys, MAX_LINEAR_SPEED.in(MetersPerSecond));
                            frontLeft.setDesiredState(SwerveModuleVelocitys[0]);
                            frontRight.setDesiredState(SwerveModuleVelocitys[1]);
                            rearLeft.setDesiredState(SwerveModuleVelocitys[2]);
                            rearRight.setDesiredState(SwerveModuleVelocitys[3]);

                            swerveModuleDesiredStatePublisher.set(SwerveModuleVelocitys);
                        })
                .withName("drive command");
    }

    /** Sets the wheels into an X formation to prevent movement. */
    public Command setX() {
        return this.run(
                () -> {
                    frontLeft.setDesiredState(new SwerveModuleVelocity(0, Rotation2d.fromDegrees(45)));
                    frontRight.setDesiredState(
                            new SwerveModuleVelocity(0, Rotation2d.fromDegrees(-45)));
                    rearLeft.setDesiredState(new SwerveModuleVelocity(0, Rotation2d.fromDegrees(-45)));
                    rearRight.setDesiredState(new SwerveModuleVelocity(0, Rotation2d.fromDegrees(45)));
                });
    }

    public ChassisVelocities getRobotRelativeSpeeds() {
        return DRIVE_KINEMATICS.toChassisVelocities(
                frontLeft.getState(),
                frontRight.getState(),
                rearLeft.getState(),
                rearRight.getState());
    }

    public void setRobotRelativeSpeeds(ChassisVelocities speeds) {
        speeds = ChassisVelocities.discretize(speeds, .02);
        var SwerveModuleVelocitys = DRIVE_KINEMATICS.toSwerveModuleVelocitys(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(
                SwerveModuleVelocitys, MAX_LINEAR_SPEED.in(MetersPerSecond));
        frontLeft.setDesiredState(SwerveModuleVelocitys[0]);
        frontRight.setDesiredState(SwerveModuleVelocitys[1]);
        rearLeft.setDesiredState(SwerveModuleVelocitys[2]);
        rearRight.setDesiredState(SwerveModuleVelocitys[3]);
        swerveModuleDesiredStatePublisher.set(SwerveModuleVelocitys);
    }

    private void configurePathPlannerLogging() {
        PathPlannerLogging.setLogCurrentPoseCallback(
                (pose) -> {
                    field2d.setRobotPose(pose);
                });

        PathPlannerLogging.setLogTargetPoseCallback(
                (pose) -> {
                    field2d.getObject("ROBOT target pose").setPose(pose);
                });

        PathPlannerLogging.setLogActivePathCallback(
                (poses) -> {
                    field2d.getObject("ROBOT path").setPoses(poses);
                });
    }

    private double getRotRate(
            double currentAngle,
            double rotRate,
            double polarXSpeed,
            double polarYSpeed,
            BooleanSupplier shouldAutoAlign) {
        if (shouldAutoAlign.getAsBoolean()) {
            return AutoAlignUtil.getAutoOrientRotRate(
                    () -> robotPose,
                    RebuiltVisionUtil.getAlignmentTargetPose(() -> robotPose),
                    TransformConstants.ROBOT_TO_SHOOTER_TRANSFORM);
        } else {
            return getHeadingCorrectionRotRate(currentAngle, rotRate, polarXSpeed, polarYSpeed);
        }
    }

    private double getHeadingCorrectionRotRate(
            double currentAngle, double rotRate, double polarXSpeed, double polarYSpeed) {
        // Debouncer ensures that there is no back-correction immediately after turning
        // Deadband for small movements - they are so slight they do not need correction
        // and correction causes robot to spasm
        double newRotRate = 0;
        if (rotationDebouncer.calculate(rotRate == 0)
                && (Math.abs(polarXSpeed) >= MIN_STRAFE_SPEED
                        || Math.abs(polarYSpeed) >= MIN_STRAFE_SPEED)) {
            newRotRate = newRotRate + drivePIDController.calculate(currentAngle, desiredAngle);
        } else {
            newRotRate = rotRate;
            desiredAngle = currentAngle;
        }
        return newRotRate;
    }

    @Override
    public Rotation2d getYaw() {
        return new Rotation2d(gyro.getYaw(false));
    }

    @Override
    public Rotation2d getYawPerSecond() {
        return new Rotation2d(gyro.getAngularVelocity().getValueAsDouble());
    }

    @Override
    public double getLinearSpeed() {
        double velocityXMPS = getRobotRelativeSpeeds().vxMetersPerSecond;
        double velocityYMPS = getRobotRelativeSpeeds().vyMetersPerSecond;
        return Math.hypot(velocityXMPS, velocityYMPS);
    }

    @Override
    public void addVisionMeasurement(
            Pose2d pose, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs) {
        // Something cursed happens here where the robot code crashes in a loop on first
        // boot, complaining about doing a Rotation2d.exp on a Rotation2d with x = y =
        // 0. I (Will) _think_ it has to do with an invalid timestamp value passed here
        // (negative? before robot boot?) that then causes the poseEstimator to try to
        // replay odometry measurements that it doesn't have. This try-catch fixes the
        // issue, and who wants vision updates to crash their robot code anyway?

        if (timestampSeconds <= 0) {
            return;
        }
        var poseEstimate = poseEstimator.getEstimatedPosition();
        if (Double.isNaN(poseEstimate.getX()) || (poseEstimate.getX() == 0)) {
            poseEstimator.resetPose(pose);
        } else {
            poseEstimator.addVisionMeasurement(pose, timestampSeconds, visionMeasurementStdDevs);
        }
    }

    // new method workflow:
    // 1. calculate the trapezoid
    // 2. periodically yoink the next trapezoid
    // 3. move the robot
    // 4. check for the goal
    // 5. end the robot

    // this method has a LOT of suppliers - so short explanation of why they're good
    // basically, when a button binding is called in robotContainer, it makes an
    // object
    // represnting the command. this object does not change, and it does not
    // actually look
    // at the command each time - it just calls the one that was constructed at
    // robotInit
    // by using a supplier, the robot knows 'hey, this value might change, so i
    // should
    // check it every time i use this object' thus allowing it to change
    public Command driveToPoseOnExecute(Supplier<Pose2d> wantPose) {
        return this.run(
                        () -> {
                            atGoal = false;
                            Supplier<Pose2d> goalPose = wantPose;

                            if (FieldConstants.alliance.isPresent()
                                    && FieldConstants.alliance.get() == Alliance.Red) {
                                goalPose = () -> FlippingUtil.flipFieldPose(wantPose.get());
                            }

                            Telemetry.log(
                                    "drive/vision/goalPoseY", goalPose.get().getY());
                            Telemetry.log(
                                    "drive/vision/goalPosex", goalPose.get().getX());
                            Telemetry.log(
                                    "drive/vision/goalTheta",
                                    goalPose.get().getRotation().getDegrees());

                            ChassisVelocities alignmentSpeeds =
                                    AutoAlignUtil.getDriveToPoseVelocities(
                                            () -> robotPose, goalPose);

                            // tolerances were accounted for in getDriveToPoseVelocities
                            atGoal =
                                    alignmentSpeeds.vxMetersPerSecond == 0
                                            && alignmentSpeeds.vyMetersPerSecond == 0
                                            && alignmentSpeeds.omegaRadiansPerSecond == 0;

                            ChassisVelocities finalAlignmentSpeeds =
                                    ChassisVelocities.fromFieldRelativeSpeeds(
                                            alignmentSpeeds, robotPose.getRotation());

                            setRobotRelativeSpeeds(finalAlignmentSpeeds);
                        })
                .until(() -> atGoal);
    }

    public Command disableDriveToPose() {
        return this.runOnce(
                () -> {
                    atGoal = true;
                    frontLeft.setDesiredState(new SwerveModuleVelocity(0, Rotation2d.fromDegrees(0)));
                    frontRight.setDesiredState(new SwerveModuleVelocity(0, Rotation2d.fromDegrees(0)));
                    rearLeft.setDesiredState(new SwerveModuleVelocity(0, Rotation2d.fromDegrees(0)));
                    rearRight.setDesiredState(new SwerveModuleVelocity(0, Rotation2d.fromDegrees(0)));
                });
    }

    private void logAndUpdateDriveSubsystemStates() {
        states.pose = getPose();
        states.poseTheta = states.pose.getRotation().getDegrees();
        states.velocityXMPS = getRobotRelativeSpeeds().vxMetersPerSecond;
        states.velocityYMPS = getRobotRelativeSpeeds().vyMetersPerSecond;
        states.totalVelocity = Math.hypot(states.velocityXMPS, states.velocityYMPS);
        states.angularVelocity = Units.radiansToDegrees(relativeRobotSpeeds.omegaRadiansPerSecond);
        states.gyroAngleDegrees = gyro.getYaw(false).in(Degrees);

        // Publish the pose in a struct that can be laid onto the "odometry" view in
        // advantagescope
        posePublisher.set(states.pose);

        SmartDashboard.putBoolean("vision/orient/shouldAutoOrient", isAutoOrienting);
        Telemetry.log(
                "shooter/distance to hub (in)",
                RebuiltVisionUtil.getDistanceToAlignmentTarget(() -> robotPose).in(Inches));
        Telemetry.log("drive/Pose X(m)", states.pose.getX());
        Telemetry.log("drive/Pose Y(m)", states.pose.getY());
        Telemetry.log("drive/Pose Theta(deg)", states.poseTheta);
        Telemetry.log("drive/Linear Velocity X(mps)", states.velocityXMPS);
        Telemetry.log("drive/Linear Velocity Y(mps)", states.velocityYMPS);
        Telemetry.log("drive/Total Velocity(mps)", states.totalVelocity);
        Telemetry.log("drive/Angular Velocity(deg per sec)", states.angularVelocity);
        Telemetry.log("drive/Gyro Angle(deg)", states.gyroAngleDegrees);
    }
}
