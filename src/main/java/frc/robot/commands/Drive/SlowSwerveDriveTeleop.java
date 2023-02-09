package frc.robot.commands.Drive;

import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.SwerveModuleWOCan;

public class SlowSwerveDriveTeleop extends SecondSwerveDriveTeleop {
    public static class Constants {
        public static final double MAX_ACCELERATION_UNITS_PER_SECOND = 3;
        public static final double MAX_ANGULAR_ACCELERATION_UINTS_PER_SECOND = 3;
        
        public static final double MAX_SPEED_METERS_PER_SECOND = SwerveModuleWOCan.Constants.PHYSICAL_MAX_SPEED_METERS_PER_SECOND / 4;
        public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND= SwerveModuleWOCan.Constants.PHYSICAL_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND / 4;
    }

    public SlowSwerveDriveTeleop(Drive drive, CommandXboxController driver, 
                                Supplier<Boolean> isFieldOriented, double multiplier) {
        super(drive, driver, isFieldOriented, multiplier);
    }
}
