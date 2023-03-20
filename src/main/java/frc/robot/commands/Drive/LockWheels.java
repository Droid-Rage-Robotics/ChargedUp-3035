package frc.robot.commands.Drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drive.Drive;

public class LockWheels extends SequentialCommandGroup {
    public LockWheels(Drive drive) {
        addCommands(
            Commands.run(() -> drive.setModuleStates(new SwerveModuleState[] {//make it do degrees
                new SwerveModuleState(0.01, new Rotation2d(Math.PI / 4)),
                new SwerveModuleState(0.01, new Rotation2d(-Math.PI / 4)),
                new SwerveModuleState(0.01, new Rotation2d(-Math.PI / 4)),
                new SwerveModuleState(0.01, new Rotation2d(Math.PI / 4))
            }), drive),
            Commands.waitSeconds(1),
            Commands.run(() -> drive.setModuleStates(new SwerveModuleState[] {
                new SwerveModuleState(0.0, new Rotation2d(Math.PI / 4)),
                new SwerveModuleState(0.0, new Rotation2d(-Math.PI / 4)),
                new SwerveModuleState(0.0, new Rotation2d(-Math.PI / 4)),
                new SwerveModuleState(0.0, new Rotation2d(Math.PI / 4))
            }), drive)
        );
    }
}
