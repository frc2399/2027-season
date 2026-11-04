package frc.robot.util;

import org.wpilib.driverstation.Alliance;

import frc.robot.constants.FieldConstants;

public class GameState {
    public static Alliance getAlliance() {
        if (FieldConstants.alliance.isEmpty()) {
            return Alliance.RED;
        }
        return FieldConstants.alliance.get();
    }
}
