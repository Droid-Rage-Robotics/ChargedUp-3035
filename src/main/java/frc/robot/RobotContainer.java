package frc.robot;

import frc.robot.subsystems.*;
import frc.robot.subsystems.EnumPositions.Position.Positions;
import frc.robot.utilities.ComplexWidgetBuilder;
import frc.robot.utilities.ShuffleboardValue;
import frc.robot.commands.Autos;
import frc.robot.commands.Drive.LockWheels;
import frc.robot.commands.Drive.SwerveDriveTeleop;
import frc.robot.commands.ElevatorCommands.*;
import frc.robot.commands.Manual.ManualPivot;
import frc.robot.commands.Manual.ToggleIntake;


import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
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
    // reset network tables button
    // auto align 
    // fixnetowrk tables!!!
    // recalibrate pigeon on start
    // why are swerves not simplifying turns
    //add Physical and digital limit switch
    // make shuffleboard stuff able to use a class instead of a string
    // safe sparkmax
    // arm class
    // FIX OUTTAKE COMMAND!!!!!!!!!!!!!!!!!!!



    // limielite
    // 

    private final Drive drive = new Drive();
    private final Elevator elevator = new Elevator();
    // private final Pivot2 pivot = new Pivot2(); 
    private final Pivot pivot = new Pivot();
    private final Intake intake = new Intake();

    private final CommandXboxController driver =
        new CommandXboxController(DroidRageConstants.Gamepad.DRIVER_CONTROLLER_PORT);
    private final CommandXboxController operator =
        new CommandXboxController(DroidRageConstants.Gamepad.OPERATOR_CONTROLLER_PORT);

    SendableChooser<CommandBase> autoChooser = new SendableChooser<CommandBase>();

    public RobotContainer() {
        
        // autoChooser.addOption("Top", Autos.top(drive, elevator, pivot, intake));
        // autoChooser.setDefaultOption("Middle", Autos.dropPlusPark(drive, elevator, pivot, intake));
        
        autoChooser.addOption("PreloadPlusPark", Autos.preloadPlusPark(drive, elevator, pivot, intake));
        autoChooser.addOption("Straight Test", Autos.straightTest(drive));
        autoChooser.addOption("Straight Test BACK", Autos.straightTestBack(drive));
        autoChooser.addOption("One: CUbe + drop", Autos.oneToCubeAndToDrop(drive, elevator, pivot, intake));
        autoChooser.addOption("three: CUbe + drop", Autos.threeToCubeAndToDrop(drive, elevator, pivot, intake));
        autoChooser.addOption("Turn Test", Autos.turnTest(drive));

        // autoChooser.addOption("Turn2", new SequentialCommandGroup(
        //     new InstantCommand(() -> drive.resetOdometry(trajectory.getInitialPose())),
        //     swerveControllerCommand,
        //     new InstantCommand(() -> {
        //             drive.stop();
        //             xController.close();
        //             yController.close();
        //         }
        //     )));
        
        // autoChooser.addOption("Preload+Drop", Autos.oneToCubeAndToDrop(drive, elevator, pivot, intake));
        autoChooser.addOption("Charge", Autos.charge(drive, elevator, pivot, intake));
        // autoChooser.addOption("Charge2", Autos.charge2(drive, elevator, pivot, intake)); - no use 
        // autoChooser.addOption("Intake", Autos.intake(drive, elevator, pivot, intake));
        // autoChooser.addOption("Strafe Left", Autos.strafeRight(drive, elevator, pivot, intake));
        // autoChooser.addOption("Strafe Right", Autos.strafeRight(drive, elevator, pivot, intake));
        ComplexWidgetBuilder.create(autoChooser, "Auto Chooser", "Misce")
            .withSize(1, 3);

            
        // new ComplexWidgetBuilder(CameraServer.startAutomaticCapture(), "USB Cam2era Stream", "Misce")
        //     .withSize(5, 5);

            // Shuffleboard.getTab("Drive").getComponents().removeAll(null);
        //     // Shuffleboard.getTab("Drive").add
        // List<ShuffleboardComponent<?>> components = Shuffleboard.getTab("Drive").getComponents();

        // for (ShuffleboardComponent<?> component: components) {
        //     if (component.getClass() == SimpleWidget.class) {
        //         ((SimpleWidget)component).close();
        //     }
        // }
        ComplexWidgetBuilder.create(CameraServer.startAutomaticCapture(), "USB Camera Stream", "Misce")
            .withSize(5, 5);
    }

    public void configureTeleOpBindings() {
        drive.resetOffsetCommand();//TODO:TEST!! - Does this run continuously or just once?
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
            .onTrue(intake.run(intake::intake)) 
            .onFalse(intake.run(intake::stop));

        driver.leftTrigger()
            .onTrue(intake.run(intake::outtake))
            .onFalse(intake.run(intake::stop));

        driver.a()
            .onTrue(drive.resetOffsetCommand());

        driver.b()
            .onTrue(
                // new MoveToPosition(elevator, pivot, intake)
                // Commands.sequence(
                    // intake.toggleCommand()
                    new ToggleIntake(elevator, pivot, intake)
                    
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
                    new MoveElevator(elevator, pivot, Positions.LOWCONE)
            );
        operator.x()
            .onTrue(
                    // new MoveMid(elevator, pivot)
                    new MoveElevator(elevator, pivot, Positions.MIDCONE)
            );
        operator.y()
            .onTrue(
                // new MoveHigh(elevator, pivot)  
                new MoveElevator(elevator, pivot, Positions.HIGHCONE)
            );
        //  operator.b()
        //     .onTrue(
        //         new SequentialCommandGroup(
        //         elevator.moveIntake2High(),
        //         pivot.moveIntake2High())
        //     );
        
        operator.povUp()
            .onTrue(
                // new MoveIntake1High(elevator, pivot)
                new MoveElevator(elevator, pivot, Positions.INTAKEHIGH1CONE)
            );
        operator.povRight()
            .onTrue(
                // new MoveIntake2High(elevator, pivot)//Make grab cone from the drop
                new MoveElevator(elevator, pivot, Positions.INTAKEHIGH2CONE)
            );
        operator.povLeft()
            .onTrue(
                // new MoveIntakeLow(elevator, pivot)//Cube and Cone
                new MoveElevator(elevator, pivot, Positions.INTAKELOWCONE)
            );
        //  operator.povRight()
        //     .onTrue(new SequentialCommandGroup(
        //         elevator.setPosition(Elevator.ElevatorPosition.LOWCUBE),
        //         pivot.setTargetPosition(PivotPosition.LOWCUBE)
        //     )//Cube
            // );
            
        operator.povDown()
            .onTrue(
                // new MoveHold(elevator, pivot)
                new MoveElevator(elevator, pivot, Positions.HOLD)
            );

        // operator.rightTrigger()
        //     .onTrue(
        //         new Outtake(elevator, pivot, intake) //TODO: fix this !!!!!!!!!!!!
        //     );
            // / make INTAKE ALLWYS RUNNING

        operator.start()
            .onTrue(elevator.resetElevatorEncoders());
      
      
      
        operator.back()
            .onTrue(pivot.resetPivotEncoder()); //Pivot is Absolute(not)

        /* 
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
