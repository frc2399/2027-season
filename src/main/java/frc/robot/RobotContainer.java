package frc.robot;

import static org.wpilib.units.Units.Degrees;

import org.wpilib.command2.Command;
import org.wpilib.command2.button.CommandXboxController;
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

    private static final CommandXboxController driverController =
            new CommandXboxController(DriveControlConstants.DRIVER_CONTROLLER_PORT);


    private boolean isGyroConfigured = false;
    private boolean autonHasNoGyroAngle = false;
    private String autonGyroConfiguredFor = "";

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

    public Command disableSubsystems() {
        return drive.disableDriveToPose();
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
        driverController.b().onTrue(commandFactory.resetHeading(Degrees.of(0)));
        // manual shooting is a backup so we can still score if there's an issue
        // with the limelight or robot pose.
        driverController.x().whileTrue(drive.setX());
    }



}
