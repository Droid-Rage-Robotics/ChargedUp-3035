package frc.robot;

import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.commands.ManualArm;
import frc.robot.commands.Drive.SwerveDriveTeleop;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
    //TODO: Ideas
    // all abort button
    // override button for when sensors fail
    // read about shuffleboard and do great things
    // find out how to make a field graph with robot pose on it and preferably a visual for path following
    // custom shuffleboard droid rage theme
    // Consider setitng drive motor to break and turn to coast
    // Bright Color on shuffleboard when block is detected
    // path weaver vs path planner
    // Desaturate wheels speeds
    // reset network tables button
    // auto align 
    
    private final Drive drive = new Drive();
    private final Elevator elevator = new Elevator();
    private final Arm arm = new Arm();
    private final Claw claw = new Claw();

    private final CommandXboxController driver =
        new CommandXboxController(DroidRageConstants.Gamepad.DRIVER_CONTROLLER_PORT);
    private final CommandXboxController operator =
        new CommandXboxController(DroidRageConstants.Gamepad.OPERATOR_CONTROLLER_PORT);

    public void configureTeleOpBindings() {
        // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
        // new Trigger(exampleSubsystem::exampleCondition)
            // .onTrue(new ExampleCommand(exampleSubsystem));

        drive.setDefaultCommand(new SwerveDriveTeleop(
            drive, 
            driver::getLeftX, 
            driver::getLeftY, 
            driver::getRightX
        ));

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
        driver.rightBumper()
            .onTrue(claw.toggleClaw()); 
    
        driver.rightTrigger()
            .onTrue(claw.intake()) 
            .onFalse(claw.stopIntake());
        driver.rightTrigger()
            .onTrue(claw.outtake())
            .onFalse(claw.stopIntake());



        //Buttons to add: Toggle Button for Cone/Cube
        arm.setDefaultCommand(new ManualArm(operator, arm));

        operator.a()
            .onTrue(
                new ParallelCommandGroup(
                    elevator.moveLow(),
                    arm.moveLow()
                )
            );
        operator.x()
            .onTrue(
                new ParallelCommandGroup(
                    elevator.moveMid(),
                    arm.moveMid()
                )
            );
        operator.y()
            .onTrue(
                new ParallelCommandGroup(
                    elevator.moveHigh(),
                    arm.moveHigh()
                )
            );
    }

    public void configureTestBindings() {
        
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
        // SwerveControllerCommand a = new SwerveControllerCommand(null, null, null, null, null, null)
        // RamseteCommand ramseteCommand = new RamseteCommand(null, null, null, null, null, null, null, null, null, null)
        
        
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
