package frc.robot;

import static org.wpilib.units.Units.Degrees;

import org.wpilib.command2.button.CommandGamepad;
import org.wpilib.math.util.MathUtil;

import frc.robot.constants.RobotConstants.DriveControlConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.gyro.Gyro;

public class RobotContainer {
    private SubsystemFactory subsystemFactory = new SubsystemFactory();
    public Gyro gyro = subsystemFactory.buildGyro();

    public DriveSubsystem drive = subsystemFactory.buildDriveSubsystem(gyro);

    // this is public because we need to run the visionPoseEstimator periodic from
    // Robot
    public CommandFactory commandFactory =
            new CommandFactory(
                    drive,
                    gyro
                    );

    private static final CommandGamepad driverController =
            new CommandGamepad(DriveControlConstants.DRIVER_CONTROLLER_PORT);


    public RobotContainer() {
        configureDefaultCommands();
        configureButtonBindingsDriver();
    }

    /**
     * Updates the simulation state, particularly the gyro. This method should be called from
     * Robot.simulationPeriodic().
     */
    public void updateSimulation() {
        if (!Robot.isSimulation()) {
            return;
        }
    }

    public void configureDefaultCommands() {
        drive.setDefaultCommand(
                drive.driveCommand(
                        () ->
                                -(MathUtil.applyDeadband(
                                        driverController.getLeftY(),
                                        DriveControlConstants.DRIVE_DEADBAND)),
                        () ->
                                -(MathUtil.applyDeadband(
                                        driverController.getLeftX(),
                                        DriveControlConstants.DRIVE_DEADBAND)),
                        () ->
                                -(MathUtil.applyDeadband(
                                        driverController.getRightX(),
                                        DriveControlConstants.DRIVE_DEADBAND)),
                        true,
                        () ->
                                (driverController.leftBumper().getAsBoolean()
                                        || driverController.leftTrigger().getAsBoolean())));
    }

    private void configureButtonBindingsDriver() {

        // note! do not bind to the left bumper button; it is used in drive command for
        // auto-orient!
        // faceRight = B on Xbox
        driverController.faceRight().onTrue(commandFactory.resetHeading(Degrees.of(0)));
        // manual shooting is a backup so we can still score if there's an issue
        // with the limelight or robot pose.
        // faceLeft = X on Xbox
        driverController.faceLeft().whileTrue(drive.setX());
    }



}
