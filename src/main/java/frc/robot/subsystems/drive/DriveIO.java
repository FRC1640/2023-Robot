package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

public interface DriveIO {
    @AutoLog
    public static class DriveIOInputs {
        
    }
    /** Updates the set of loggable inputs. */
    public default void updateInputs(DriveIOInputs inputs) {

    }
}
