<<<<<<< Updated upstream
=======
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

>>>>>>> Stashed changes
package frc.robot;

import static org.wpilib.units.Units.Degrees;

import org.wpilib.command2.button.CommandGamepad;
import org.wpilib.math.util.MathUtil;

import frc.robot.constants.RobotConstants.DriveControlConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.gyro.Gyro;
import org.wpilib.telemetry.Telemetry;
import org.wpilib.math.util.MathUtil;
import org.wpilib.units.measure.Angle;
import org.wpilib.util.Alert;
import org.wpilib.util.Alert.Level;
import org.wpilib.util.AlertDataJNI;
import org.wpilib.driverstation.DriverStation;
import org.wpilib.driverstation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.button.CommandXboxController;
import frc.robot.constants.FieldConstants;
import frc.robot.util.FieldCalculationHelpers;
import frc.robot.vision.VisionPoseEstimator;
import java.util.Optional;

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
        private static final CommandXboxController tuningController =
            new CommandXboxController(DriveControlConstants.TUNING_CONTROLLER_PORT);

    public VisionPoseEstimator visionPoseEstimator =
            new VisionPoseEstimator(drive, subsystemFactory.getRobotType());
    
    private Command defaultCommand = Commands.none();

    private final Alert driverDisconnected =
            new Alert("driverDisconnected", "Driver controller disconnected!", Alert.Level.MEDIUM);
    private final Alert noAutonSelectedAlert =
            new Alert("noAutonSelected", "Auton is not selected!", Alert.Level.MEDIUM);
    private final Alert lowBatteryAlert =
            new Alert(
                    "lowBatteryVoltage", "Battery voltage is very low, turn off the robot or replace the battery to avoid damage.",
                    Alert.Level.MEDIUM);
    private final Alert gyroNeedsConfiguringAlert =
            new Alert(
                    "gyroNeedsConfiguring", "Gyro not configured! Auton + vision will not work; click the button below auton selector.",
                    Alert.Level.MEDIUM);
    private final Alert autonHasNoGyroAngleAlert =
            new Alert("autonHasNoGyroAngle", "Selected auton will not configure gyro properly!", Alert.Level.MEDIUM);
                //todo: highkey no auton need to change this later
    private boolean isGyroConfigured = false;
    private boolean autonHasNoGyroAngle = false;
    private String autonGyroConfiguredFor = "";

    public RobotContainer() {
        configureDefaultCommands();
        configureButtonBindingsDriver();
        configureButtonBindingsTuningController();

        Telemetry.log("robot/driverController", driverController.getHID());
    }

    /**
     * Updates the simulation state, particularly the gyro. This method should be called from
     * Robot.simulationPeriodic().
     */
    public void updateSimulation() {
        if (!Robot.isSimulation()) {
            return;
        }

        // Check if gyro is using the simulation implementation
        if (gyro.getIO() instanceof frc.robot.subsystems.drive.gyro.GyroSim) {
            frc.robot.subsystems.drive.gyro.GyroSim gyroSim =
                    (frc.robot.subsystems.drive.gyro.GyroSim) gyro.getIO();

            // Get the robot's current angular velocity from the drive subsystem
            // The drive subsystem calculates this from the swerve module states
            double omegaRadPerSec = drive.getRobotRelativeSpeeds().omegaRadiansPerSecond;

            // Update the simulated gyro with the angular velocity
            gyroSim.setAngularVelocity(omegaRadPerSec);

            // Integrate to update the heading (20ms = 0.02 seconds)
            gyroSim.update(0.02);
        }
    }

    public void disableSubsystems() {
        drive.disableDriveToPose();
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



        driverController.b().onTrue(commandFactory.resetHeading(Degrees.of(0)));
        driverController.x().whileTrue(drive.setX());
    }

    private void configureButtonBindingsTuningController() {
        
        tuningController.povDown().onTrue(commandFactory.resetHeading(Degrees.of(180)));
    }
    private void setUpAuton() {

        // adds the different auton programs to the list of options on the
        // SmartDashboard
        Telemetry.log(
                "Autos/configure gyro (CHOOSE AUTON THEN CLICK ME!)", resetGyroByAuton());
        Telemetry.log("alliance/reset blue", resetAllianceBlue());
        Telemetry.log("alliance/reset red", resetAllianceRed());
    }

    public Command resetAllianceBlue() {
        return Commands.runOnce(
                        () -> {
                            FieldConstants.alliance = Optional.of(Alliance.BLUE);
                            isGyroConfigured = false;
                        })
                .ignoringDisable(true);
    }

    public Command resetAllianceRed() {
        return Commands.runOnce(
                        () -> {
                            FieldConstants.alliance = Optional.of(Alliance.RED);
                            isGyroConfigured = false;
                        })
                .ignoringDisable(true);
    }

    public void setAlerts() {
        lowBatteryAlert.set(
                (RobotController.getBatteryVoltage() > 0.0
                        && RobotController.getBatteryVoltage()
                                <= DriveControlConstants.LOW_BATTERY_VOLTAGE));
        driverDisconnected.set(
                !DriverStation.isJoystickConnected(driverController.getHID().getPort()));
    }
}
