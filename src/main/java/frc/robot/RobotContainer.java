package frc.robot;

import frc.robot.subsystems.Drive;
import frc.robot.subsystems.VerticalExtension;
import frc.robot.commands.Drive.SwerveDriveTeleop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
    private final Drive drive = new Drive();
    // private final HorizontalExtension horizontalExtension = new HorizontalExtension();
    // private final VerticalExtension verticalExtension = new VerticalExtension();

    private final CommandXboxController driver =
        new CommandXboxController(DroidRageConstants.Gamepad.DRIVER_CONTROLLER_PORT);
    private final CommandXboxController operator =
        new CommandXboxController(DroidRageConstants.Gamepad.OPERATOR_CONTROLLER_PORT);

    public RobotContainer() {
        // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
        // new Trigger(exampleSubsystem::exampleCondition)
            // .onTrue(new ExampleCommand(exampleSubsystem));

        // verticalExtension.setDefaultCommand(verticalExtension.moveToPosition());

        drive.setDefaultCommand(new SwerveDriveTeleop(
            drive, 
            driver::getLeftX, 
            driver::getLeftY, 
            driver::getRightX,
            drive::isFieldOriented)
        );

        driver.rightBumper()
            .onTrue(drive.setTurboSpeed())
            .onFalse(drive.setNormalSpeed());
        
        driver.leftBumper()
            .onTrue(drive.setSlowSpeed())
            .onFalse(drive.setNormalSpeed());

        driver.a()
            .onTrue(drive.resetHeading());

        driver.back()
            .onTrue(drive.toggleFieldOriented());
    }
    
    public Command getAutonomousCommand() {
        // An example command will be run in autonomous
        // return Autos.exampleAuto(exampleSubsystem);
        
        // TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
        //   Drive.Constants.Auto.MAX_SPEED_METERS_PER_SECOND, 
        //   Drive.Constants.Auto.MAX_ACCELERATION_METERS_PER_SECOND)
        //     .setKinematics(Drive.Constants.DRIVE_KINEMATICS);
        
        // Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        //   new Pose2d(),
        //   List.of(
        //     new Translation2d(1,0),
        //     new Translation2d(1, -1)
        //   ), 
        //   new Pose2d(2, -1, Rotation2d.fromDegrees(180)),
        //   trajectoryConfig
        //   );
        
        // PIDController xController = new PIDController(Drive.Constants.Auto.TRANSLATIONAL_KP, 0, 0);
        // PIDController yController = new PIDController(Drive.Constants.Auto.TRANSLATIONAL_KP, 0, 0);
        // ProfiledPIDController thetaController = new ProfiledPIDController(
        //   Drive.Constants.Auto.THETA_KP, 
        //   0, 
        //   0, 
        //   Drive.Constants.Auto.THETA_CONSTRAINTS);
        // thetaController.enableContinuousInput(-Math.PI, Math.PI);
        
        // SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        //   trajectory,
        //   drive::getPose,
        //   Drive.Constants.DRIVE_KINEMATICS,
        //   xController,
        //   yController,
        //   thetaController,
        //   drive::setModuleStates,
        //   drive
        // );
        
        
        // return new SequentialCommandGroup(
        //   new InstantCommand(() -> drive.resetOdometry(trajectory.getInitialPose())),
        //   swerveControllerCommand,
        //   new InstantCommand(() -> {
        //       drive.stop();
        //       xController.close();
        //       yController.close();
        //     }
        //   )
        // );
        return new InstantCommand();
    }
}
