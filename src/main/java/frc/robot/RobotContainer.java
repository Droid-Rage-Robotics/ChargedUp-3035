package frc.robot;

import frc.robot.subsystems.*;
import frc.robot.utilities.ComplexWidgetBuilder;
import frc.robot.commands.Autos;
import frc.robot.commands.Drive.LockWheels;
import frc.robot.commands.Drive.SwerveDriveTeleop;
import frc.robot.commands.ElevatorCommands.*;
import frc.robot.commands.Manual.ManualPivot;

import java.util.Collection;
import java.util.List;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
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
    // make shuffleboard stuff able to use a class instead of a string



    // limielite
    // 

    private final Drive drive = new Drive();
    private final Elevator elevator = new Elevator();
    // private final Pivot2 pivot = new Pivot2(); 
    private final Pivot pivot =new Pivot();
    private final Intake intake = new Intake();

    private final CommandXboxController driver =
        new CommandXboxController(DroidRageConstants.Gamepad.DRIVER_CONTROLLER_PORT);
    private final CommandXboxController operator =
        new CommandXboxController(DroidRageConstants.Gamepad.OPERATOR_CONTROLLER_PORT);

    SendableChooser<CommandBase> autoChooser = new SendableChooser<CommandBase>();

    public RobotContainer() {
        
        // autoChooser.addOption("Top", Autos.top(drive, elevator, pivot, intake));
        // autoChooser.setDefaultOption("Middle", Autos.dropPlusPark(drive, elevator, pivot, intake));

         
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
            Drive.AutoConfig.MAX_SPEED_METERS_PER_SECOND.value.get(), 
            Drive.AutoConfig.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED.value.get())
                .setKinematics(Drive.SwerveConstants.DRIVE_KINEMATICS);
        
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(),
            List.of(
                new Translation2d(1,0)
            ), 
            new Pose2d(1, 1, Rotation2d.fromDegrees(180)),
            trajectoryConfig
        );
        
        PIDController xController = new PIDController(Drive.AutoConfig.TRANSLATIONAL_KP.value.get(), 0, 0);
        PIDController yController = new PIDController(Drive.AutoConfig.TRANSLATIONAL_KP.value.get(), 0, 0);
        ProfiledPIDController thetaController = new ProfiledPIDController(
            Drive.AutoConfig.THETA_KP.value.get(), 
              0, 
              0, 
            drive.getThetaConstraints());
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        
        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
            trajectory,
            drive::getPose,
            Drive.SwerveConstants.DRIVE_KINEMATICS,
            xController,
            yController,
            thetaController,
            drive::setModuleStates,
            drive
        );

        autoChooser.addOption("Straight Test", Autos.straightTest(drive));
        autoChooser.addOption("Turn Test", Autos.turnTest(drive));
        autoChooser.addOption("Turn2", new SequentialCommandGroup(
            new InstantCommand(() -> drive.resetOdometry(trajectory.getInitialPose())),
            swerveControllerCommand,
            new InstantCommand(() -> {
                    drive.stop();
                    xController.close();
                    yController.close();
                }
            )));
        // autoChooser.addOption("Preload+Drop", Autos.oneToCubeAndToDrop(drive, elevator, pivot, intake));
        autoChooser.addOption("Charge", Autos.charge(drive, elevator, pivot, intake));

        // autoChooser.addOption("Intake", Autos.intake(drive, elevator, pivot, intake));
        // autoChooser.addOption("Strafe Left", Autos.strafeRight(drive, elevator, pivot, intake));
        // autoChooser.addOption("Strafe Right", Autos.strafeRight(drive, elevator, pivot, intake));
        new ComplexWidgetBuilder(autoChooser, "Auto Chooser", "Misce")
            .withSize(1, 3);

        new ComplexWidgetBuilder(CameraServer.startAutomaticCapture(), "USB Camera Stream", "Misce")
            .withSize(5, 5);

            // Shuffleboard.getTab("Drive").getComponents().removeAll(null);
        //     // Shuffleboard.getTab("Drive").add
        // List<ShuffleboardComponent<?>> components = Shuffleboard.getTab("Drive").getComponents();

        // for (ShuffleboardComponent<?> component: components) {
        //     if (component.getClass() == SimpleWidget.class) {
        //         ((SimpleWidget)component).close();
        //     }
        // }
    }

    public void configureTeleOpBindings() {
         /*
         * Driver Controls
         */

        drive.setDefaultCommand(
            new SwerveDriveTeleop(
                drive, 
                driver::getLeftX, 
                driver::getLeftY, 
                driver::getRightX,
                driver.x()
                )
            );

        driver.rightBumper()
            .onTrue(drive.setSlowSpeed())
            .onFalse(drive.setNormalSpeed());


        driver.rightTrigger()
            .onTrue(intake.runIntake()) 
            .onFalse(intake.runHoldIntake()
            );

        driver.leftTrigger()
            .onTrue(intake.runOuttake())
            .onFalse(intake.runStop());

        driver.a()
            .onTrue(drive.resetHeading()
            );

        driver.b()
            .onTrue(
                // new MoveToPosition(elevator, pivot, intake)
                // Commands.sequence(
                    intake.toggleCommand()
                    
                // )
            ); 

        driver.back()
            .onTrue(drive.toggleFieldOriented()
            );
        
        driver.y()
            .onTrue(drive.toggleBreakMode());

        // driver.x()
        //     .onTrue(new LockWheels(drive));


        /*
         * Operator Controls
         */
        pivot.setDefaultCommand(new ManualPivot(operator::getLeftY, pivot));
        // elevator.setDefaultCommand(new ManualElevator(operator::getRightX, operator::getRightY, elevator));

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
            .onTrue(
                new MoveIntakeHigh(elevator, pivot)
            );
        operator.povLeft()
            .onTrue(
                new MoveIntakeLow(elevator, pivot)
            );
        operator.povDown()
            .onTrue(
                new MoveHold(elevator, pivot)
            );

        operator.rightTrigger()
            .onTrue(
                new Outtake(elevator, pivot, intake)
            );
            // / make INTAKE ALLWYS RUNNING

        operator.start()
            .onTrue(elevator.resetElevatorEncoders());
      
      
      
        /*operator.back()
            .onTrue(pivot.resetPivotEncoder()); //Pivot is Absolute

        
            pivot.setPowerC(-operator.getLeftY())
        operator.rightBumper()
            .onTrue(pivot.setPowerC(0.4))
            .onFalse(pivot.setPowerC(0));
        
        operator.leftBumper()
            .onTrue(pivot.setPowerC(-0.4))
            .onFalse(pivot.setPowerC(0));*/

        // operator.leftTrigger()
        //     .onTrue(
        //         new SequentialCommandGroup(
        //             new MoveMid(elevator,pivot),
        //             new WaitCommand(3),
        //             new DropCone(elevator, pivot, intake)
        //         )
        //     );
    }

    public void configureTestBindings(){}
    
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
