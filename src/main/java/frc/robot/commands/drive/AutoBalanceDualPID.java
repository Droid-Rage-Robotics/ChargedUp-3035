package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystem.drive.Drive;

public class AutoBalanceDualPID extends CommandBase {
    private final Drive drive;
    public AutoBalanceDualPID(Drive drive) {
        this.drive = drive;
    }
}
