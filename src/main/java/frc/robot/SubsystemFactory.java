package frc.robot;

import org.wpilib.telemetry.Telemetry;

import org.wpilib.framework.RobotBase;
import frc.robot.constants.RobotConstants.MotorIdConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.SwerveModule;
import frc.robot.subsystems.drive.SwerveModuleHardwareVortex;
import frc.robot.subsystems.drive.SwerveModulePlacebo;
import frc.robot.subsystems.drive.SwerveModuleSim;
import frc.robot.subsystems.drive.gyro.Gyro;
import frc.robot.subsystems.drive.gyro.GyroHardware;
import frc.robot.subsystems.drive.gyro.GyroPlacebo;
import frc.robot.subsystems.drive.gyro.GyroSim;

public class SubsystemFactory {
    private static final double FRONT_LEFT_CHASSIS_ANGULAR_OFFSET = -Math.PI / 2;
    private static final double FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET = 0;
    private static final double REAR_LEFT_CHASSIS_ANGULAR_OFFSET = Math.PI;
    private static final double REAR_RIGHT_CHASSIS_ANGULAR_OFFSET = Math.PI / 2;

    

    public enum RobotType {
        SIM,
        ALPHA
    }

    private static RobotType robotType;



    public SubsystemFactory() {
        if (RobotBase.isSimulation()) {
            robotType = RobotType.SIM;
            
        } else {
           robotType = RobotType.ALPHA; 
        }

       Telemetry.log("robot/robotType", robotType.name());
    }

    public static RobotType getRobotType() {
        return robotType;
    }

    public DriveSubsystem buildDriveSubsystem(Gyro gyro) {
        SwerveModule frontLeft;
        SwerveModule rearLeft;
        SwerveModule frontRight;
        SwerveModule rearRight;
        if (robotType == RobotType.ALPHA) {
            frontLeft =
                    new SwerveModule(
                            new SwerveModuleHardwareVortex(
                                    MotorIdConstants.FRONT_LEFT_DRIVING_CAN_ID,
                                    MotorIdConstants.FRONT_LEFT_TURNING_CAN_ID,
                                    FRONT_LEFT_CHASSIS_ANGULAR_OFFSET,
                                    "front left"));
            frontRight =
                    new SwerveModule(
                            new SwerveModuleHardwareVortex(
                                    MotorIdConstants.FRONT_RIGHT_DRIVING_CAN_ID,
                                    MotorIdConstants.FRONT_RIGHT_TURNING_CAN_ID,
                                    FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET,
                                    "front right"));
            rearLeft =
                    new SwerveModule(
                            new SwerveModuleHardwareVortex(
                                    MotorIdConstants.REAR_LEFT_DRIVING_CAN_ID,
                                    MotorIdConstants.REAR_LEFT_TURNING_CAN_ID,
                                    REAR_LEFT_CHASSIS_ANGULAR_OFFSET,
                                    "rear left"));
            rearRight =
                    new SwerveModule(
                            new SwerveModuleHardwareVortex(
                                    MotorIdConstants.REAR_RIGHT_DRIVING_CAN_ID,
                                    MotorIdConstants.REAR_RIGHT_TURNING_CAN_ID,
                                    REAR_RIGHT_CHASSIS_ANGULAR_OFFSET,
                                    "rear right"));
            return new DriveSubsystem(frontLeft, frontRight, rearLeft, rearRight, gyro);
        } else if (robotType == RobotType.SIM) {
            frontLeft =
                    new SwerveModule(
                            new SwerveModuleSim(FRONT_LEFT_CHASSIS_ANGULAR_OFFSET, "front left"));
            frontRight =
                    new SwerveModule(
                            new SwerveModuleSim(FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET, "front right"));
            rearLeft =
                    new SwerveModule(
                            new SwerveModuleSim(REAR_LEFT_CHASSIS_ANGULAR_OFFSET, "rear left"));
            rearRight =
                    new SwerveModule(
                            new SwerveModuleSim(REAR_RIGHT_CHASSIS_ANGULAR_OFFSET, "rear right"));
            return new DriveSubsystem(frontLeft, frontRight, rearLeft, rearRight, gyro);
        } else {
            frontLeft = new SwerveModule(new SwerveModulePlacebo());
            frontRight = new SwerveModule(new SwerveModulePlacebo());
            rearLeft = new SwerveModule(new SwerveModulePlacebo());
            rearRight = new SwerveModule(new SwerveModulePlacebo());
            return new DriveSubsystem(frontLeft, frontRight, rearLeft, rearRight, gyro);
        }
    }

    public Gyro buildGyro() {
        // Use simulated gyro in simulation mode
        if (Robot.isSimulation()) {
            return new Gyro(new GyroSim());
        } else if (robotType == RobotType.ALPHA) {
            return new Gyro(new GyroHardware());
        } else {
            return new Gyro(new GyroPlacebo());
        }
    }
}
