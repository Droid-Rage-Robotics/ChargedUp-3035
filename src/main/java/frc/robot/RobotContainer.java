package frc.robot;

import frc.robot.subsystems.*;
import frc.robot.commands.Drive.SwerveDriveTeleop;
import frc.robot.commands.ElevatorCommands.MoveHigh;
import frc.robot.commands.ElevatorCommands.MoveHold;
import frc.robot.commands.ElevatorCommands.MoveIntakeHigh;
import frc.robot.commands.ElevatorCommands.MoveIntakeLow;
import frc.robot.commands.ElevatorCommands.MoveLow;
import frc.robot.commands.ElevatorCommands.MoveMid;
import frc.robot.commands.ElevatorCommands.Outtake;
import frc.robot.commands.Manual.ManualElevator;
import frc.robot.commands.Manual.ManualPivot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
    //TODO: Ideas
    // all abort button
    // override button for when sensors fail
    // read about shuffleboard and do great things
    // find out how to make a field graph with robot pose on it and preferably a visual for path following
    // custom shuffleboard droid rage theme
    // Consider setitng drive motor to break and turn to coast
    // ----Bright Color on shuffleboard when block is detected
    // ----path weaver vs path planner
    // ----Desaturate wheels speeds
    // reset network tables button
    // auto align 
    // fixnetowrk tables!!!
    // recalibrate pigeon on start
    // why are swerves not simplifying turns
    //add Physical and digital limit switch


    // limielite
    // fix drive directions for atuos/teleop
    // 

    private final Drive drive = new Drive();
    private final Elevator elevator = new Elevator();
    private final Pivot pivot = new Pivot(); 
    private Intake intake = new Intake();

    private final CommandXboxController driver =
        new CommandXboxController(DroidRageConstants.Gamepad.DRIVER_CONTROLLER_PORT);
    private final CommandXboxController operator =
        new CommandXboxController(DroidRageConstants.Gamepad.OPERATOR_CONTROLLER_PORT);

    public void configureTeleOpBindings() {
         /*
         * Driver Controls
         */

        drive.setDefaultCommand(
            new SwerveDriveTeleop(
                drive, 
                driver::getLeftX, 
                driver::getLeftY, 
                driver::getRightX
                )
            );

        driver.rightBumper()
            .onTrue(drive.setSlowSpeed())
            .onFalse(drive.setNormalSpeed());

        driver.leftBumper()
            .onTrue(drive.setSupserSlowSpeed())
            .onFalse(drive.setNormalSpeed()
            );

        driver.rightTrigger()
            .onTrue(intake.runIntake()) 
            .onFalse(intake.runStop()
            );

        driver.leftTrigger()
            .onTrue(intake.runOuttake())
            .onFalse(intake.runStop()
            );

        driver.a()
            .onTrue(drive.resetHeading()
            );

        driver.b()
            .onTrue(intake.runToggleOpen()
            ); 

        driver.back()
            .onTrue(drive.toggleFieldOriented()
            );
        
        driver.povUp()
            .onTrue(drive.toggleAntiTipping()
            );

        driver.povDown()
            .onTrue(drive.toggleAutoBalance()
            );

        /*
         * Operator Controls
         */
        pivot.setDefaultCommand(new ManualPivot(operator, pivot));
        elevator.setDefaultCommand(new ManualElevator(operator, elevator));

        operator.a()
            .onTrue(
                    new MoveLow(elevator, pivot)
            );
        operator.x()
            .onTrue(
                    new MoveMid(elevator, pivot)
            );
        operator.y()
            .onTrue(
                new MoveHigh(elevator, pivot)  
            );
        
        operator.povUp()
            .onTrue(new MoveIntakeHigh(elevator, pivot)
            );
        operator.povDown()
            .onTrue(new MoveIntakeLow(elevator, pivot)
            );
        operator.povLeft()
            .onTrue(new MoveHold(elevator, pivot)
            );

        operator.rightTrigger()
            .onTrue(new Outtake(elevator, pivot, intake)
            );
    }

    public void configureTestBindings() {
        
    }
    
    public Command getAutonomousCommand() {
        return new InstantCommand();
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
        // List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup(
        //     "First", 
        //     new PathConstraints(0.3, 0.5)
        // );

        // // This is just an example event map. It would be better to have a constant, global event map
        // // in your code that will be used by all path following commands.
        // HashMap<String, Command> eventMap = new HashMap<>();
        // eventMap.put("marker1", new PrintCommand("Passed marker 1"));

        // // Create the AutoBuilder. This only needs to be created once when robot code starts, not every time you want to create an auto command. A good place to put this is in RobotContainer along with your subsystems.
        // SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
        //     drive::getPose, // Pose2d supplier
        //     drive::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
        //     Drive.SwerveConstants.DRIVE_KINEMATICS, // SwerveDriveKinematics
        //     new PIDConstants(Drive.AutoConfig.TRANSLATIONAL_KP.value.get(), 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
        //     new PIDConstants(Drive.AutoConfig.THETA_KP.value.get(), 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
        //     drive::setModuleStates, // Module states consumer used to output to the drive subsystem
        //     eventMap,
        //     true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
        //     drive // The drive subsystem. Used to properly set the requirements of path following commands
        // );

        // //TODO check if there are any other better constructors ^^

        // Command fullAuto = autoBuilder.fullAuto(pathGroup);

        // return fullAuto;

    }
}
