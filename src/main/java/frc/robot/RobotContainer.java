package frc.robot;

import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.PIvot;
// import frc.robot.subsystems.Claw;
import frc.robot.commands.ManualArm;
import frc.robot.commands.ManualElevator;
import frc.robot.commands.Drive.SwerveDriveTeleop;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.trajectory.Trajectory;
// import edu.wpi.first.math.trajectory.TrajectoryConfig;
// import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
    public RobotContainer() {
        Drive.TeleOpConstants.MAX_ACCELERATION_UNITS_PER_SECOND.get();
        Drive.TeleOpConstants.MAX_ANGULAR_ACCELERATION_UINTS_PER_SECOND.get();
    }
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
    // fixnetowrk tables!!!
    // recalibrate pigeon on start
    // why are swerves not simplifying turns

    private final Drive drive = new Drive();

    private final Elevator elevator = new Elevator();
    private final PIvot arm = new PIvot();
    // private final Claw claw = new Claw();

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
            .onFalse(drive.setTurboSpeed());
        driver.leftBumper()
            .onTrue(drive.setSlowSpeed())
            .onFalse(drive.setTurboSpeed());

        driver.a()
            .onTrue(drive.resetHeading());

        driver.back()
            .onTrue(drive.toggleFieldOriented());
        driver.povUp()
            .onTrue(drive.toggleAntiTipping());
        driver.povDown()
            .onTrue(drive.toggleAutoBalance());
        driver.y()
            .onTrue(
                new SequentialCommandGroup(
                    elevator.moveIntakeLow(),
                    arm.moveIntakeLow()
                    // claw.intake
                )
            )
            .onFalse(
                new SequentialCommandGroup(
                    arm.moveHold()
                    // claw.outtake
                )
            );
        driver.x()
            .onTrue(
                new SequentialCommandGroup(
                    elevator.moveIntakeHigh(),
                    arm.moveIntakeHigh()
                    // claw.intake
                )
            )
            .onFalse(
                new SequentialCommandGroup(
                    elevator.moveMid(),
                    arm.moveHold()
                    // claw.outtake
                )
            );

    //Wants a button to get cone off ground
    //         .onTrue(claw.toggleClaw()); 
    //     driver.rightTrigger()
    //         .onTrue(claw.intake()) 
    //         .onFalse(claw.stopIntake());
    //     driver.rightTrigger()
    //         .onTrue(claw.outtake())
    //         .onFalse(claw.stopIntake());



        //Buttons to add: Toggle Button for Cone/Cube
        arm.setDefaultCommand(new ManualArm(operator, arm));
        elevator.setDefaultCommand(new ManualElevator(operator, elevator));

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
        operator.povUp()
            .onTrue(elevator.moveIntakeHigh()
            );
        // operator.povDown()
        //     .onTrue(elevator.moveIntakeLow()
        //     );
    //     //Buttons to add: Toggle Button for Cone/Cube
    //     arm.setDefaultCommand(new ManualArm(operator, arm));

    //     operator.a()
    //         .onTrue(
    //             new ParallelCommandGroup(
    //                 elevator.moveLow(),
    //                 arm.moveLow()
    //             )
    //         );
    //     operator.x()
    //         .onTrue(
    //             new ParallelCommandGroup(
    //                 elevator.moveMid(),
    //                 arm.moveMid()
    //             )
    //         );
    //     operator.y()
    //         .onTrue(
    //             new ParallelCommandGroup(
    //                 elevator.moveHigh(),
    //                 arm.moveHigh()
    //             )
    //         );
    }

    public void configureTestBindings() {
        
    }
    
    public Command getAutonomousCommand() {
        // // An example command will be run in autonomous
        // // return Autos.exampleAuto(exampleSubsystem);
        
        // TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
        //     Drive.AutoConstants.getMaxSpeedMetersPerSecond(), 
        //     Drive.AutoConstants.getMaxAccelerationMetersPerSecondSquared())
        //         .setKinematics(Drive.SwerveConstants.DRIVE_KINEMATICS);
        
        // Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        //     new Pose2d(),
        //     List.of(
        //         new Translation2d(1,0),
        //         new Translation2d(1, -1)
        //     ), 
        //     new Pose2d(2, -1, Rotation2d.fromDegrees(180)),
        //     trajectoryConfig
        // );
        
        // PIDController xController = new PIDController(Drive.AutoConstants.getTranslationalKp(), 0, 0);
        // PIDController yController = new PIDController(Drive.AutoConstants.getTranslationalKp(), 0, 0);
        // ProfiledPIDController thetaController = new ProfiledPIDController(
        //     Drive.AutoConstants.getThetaKp(), 
        //       0, 
        //       0, 
        //     Drive.AutoConstants.getThetaConstraints());
        // thetaController.enableContinuousInput(-Math.PI, Math.PI);
        
        // SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        //     trajectory,
        //     drive::getPose,
        //     Drive.SwerveConstants.DRIVE_KINEMATICS,
        //     xController,
        //     yController,
        //     thetaController,
        //     drive::setModuleStates,
        //     drive
        // );
        
        
        // return new SequentialCommandGroup(
        //     new InstantCommand(() -> drive.resetOdometry(trajectory.getInitialPose())),
        //     swerveControllerCommand,
        //     new InstantCommand(() -> {
        //             drive.stop();
        //             xController.close();
        //             yController.close();
        //         }
        //     )
        // );

        // This will load the file "FullAuto.path" and generate it with a max velocity of 4 m/s and a max acceleration of 3 m/s^2
        // for every path in the group
        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup(
            "First", 
            new PathConstraints(0.3, 0.5)
        );

        // This is just an example event map. It would be better to have a constant, global event map
        // in your code that will be used by all path following commands.
        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("marker1", new PrintCommand("Passed marker 1"));

        // Create the AutoBuilder. This only needs to be created once when robot code starts, not every time you want to create an auto command. A good place to put this is in RobotContainer along with your subsystems.
        SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
            drive::getPose, // Pose2d supplier
            drive::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
            Drive.SwerveConstants.DRIVE_KINEMATICS, // SwerveDriveKinematics
            new PIDConstants(Drive.AutoConstants.getTranslationalKp(), 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
            new PIDConstants(Drive.AutoConstants.getThetaKp(), 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
            drive::setModuleStates, // Module states consumer used to output to the drive subsystem
            eventMap,
            true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
            drive // The drive subsystem. Used to properly set the requirements of path following commands
        );

        //TODO check if there are any other better constructors ^^

        Command fullAuto = autoBuilder.fullAuto(pathGroup);

        return fullAuto;

    }
}
