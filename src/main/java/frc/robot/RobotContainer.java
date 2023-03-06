package frc.robot;

import frc.robot.subsystems.*;
import frc.robot.utilities.ComplexWidgetBuilder;
import frc.robot.commands.Autos;
import frc.robot.commands.Drive.LockWheels;
import frc.robot.commands.Drive.SwerveDriveTeleop;
import frc.robot.commands.ElevatorCommands.DropCone;
import frc.robot.commands.ElevatorCommands.MoveHigh;
import frc.robot.commands.ElevatorCommands.MoveHold;
import frc.robot.commands.ElevatorCommands.MoveIntakeHigh;
import frc.robot.commands.ElevatorCommands.MoveIntakeLow;
import frc.robot.commands.ElevatorCommands.MoveLow;
import frc.robot.commands.ElevatorCommands.MoveMid;
import frc.robot.commands.ElevatorCommands.MoveToPosition;
import frc.robot.commands.ElevatorCommands.Outtake;
import frc.robot.commands.Manual.ManualElevator;
import frc.robot.commands.Manual.ManualPivot;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
    // fix drive directions for atuos/teleop
    // 

    private final Drive drive = new Drive();
    private final Elevator elevator = new Elevator();
    private final Pivot pivot = new Pivot(); 
    private final Intake intake = new Intake();

    private final CommandXboxController driver =
        new CommandXboxController(DroidRageConstants.Gamepad.DRIVER_CONTROLLER_PORT);
    private final CommandXboxController operator =
        new CommandXboxController(DroidRageConstants.Gamepad.OPERATOR_CONTROLLER_PORT);

    SendableChooser<CommandBase> autoChooser = new SendableChooser<CommandBase>();

    public RobotContainer() {
        
        // autoChooser.addOption("Top", Autos.top(drive, elevator, pivot, intake));
        autoChooser.setDefaultOption("Middle", Autos.dropPlusPark(drive, elevator, pivot, intake));
        autoChooser.addOption("Straight Test", Autos.straightTest(drive));
        autoChooser.addOption("Turn Test", Autos.turnTest(drive));
        autoChooser.addOption("Preload+Drop", Autos.preloadPlusDrop(drive, elevator, pivot, intake));
        autoChooser.addOption("Charge", Autos.charge(drive, elevator, pivot, intake));

        // autoChooser.addOption("Intake", Autos.intake(drive, elevator, pivot, intake));
        // autoChooser.addOption("Strafe Left", Autos.strafeRight(drive, elevator, pivot, intake));
        // autoChooser.addOption("Strafe Right", Autos.strafeRight(drive, elevator, pivot, intake));
        new ComplexWidgetBuilder(autoChooser, "Auto Chooser", "Misc")
            .withSize(1, 3);

        new ComplexWidgetBuilder(CameraServer.startAutomaticCapture(), "USB Camera Stream", "Misc")
            .withSize(5, 5);
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
                driver::getRightX
                )
            );

        driver.rightBumper()
            .onTrue(drive.setSlowSpeed())
            .onFalse(drive.setNormalSpeed());

        // driver.leftBumper()
        //     .onTrue(drive.setTurboSpeed())
        //     .onFalse(drive.setNormalSpeed()
        //     );

        driver.rightTrigger()
            .onTrue(intake.runIntake()) 
            .onFalse(intake.runHoldIntake()
            );

        driver.leftTrigger()
            .onTrue(intake.runOuttake())
            .onFalse(intake.runStop()
            );

        driver.a()
            .onTrue(drive.resetHeading()
            );

        driver.b()
            .onTrue(
                new SequentialCommandGroup(
                    intake.runToggleOpen(),
                    new MoveToPosition(elevator, pivot)
                )
            ); 

        driver.back()
            .onTrue(drive.toggleFieldOriented()
            );
        
        driver.y()
            .onTrue(drive.toggleBreakMode());

        driver.x()
            .onTrue(LockWheels.create(drive));
        // driver.povUp()
        //     .onTrue(drive.toggleAntiTipping()
        //     );

        // driver.povDown()
        //     .onTrue(drive.toggleAutoBalance()
        //     );

        /*
         * Operator Controls
         */
        // pivot.setDefaultCommand(Commands.repeatingSequence(
        //     pivot.setCurrentPositionManually(pivot.getTargetPosition()+operator.getLeftY()*1)
        // ));
        // elevator.setDefaultCommand(new ManualElevator(operator, elevator));

        // new Trigger(() -> Math.abs(-operator.getLeftY()) > 0.1)
            // .whileTrue(pivot.setCurrentPositionManually(() -> pivot.getTargetPosition() + (-operator.getLeftY())));

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
            .onTrue(new MoveIntakeHigh(elevator, pivot)
            );
        operator.povLeft()
            .onTrue(new MoveIntakeLow(elevator, pivot)
            );
        operator.povDown()
            .onTrue(new MoveHold(elevator, pivot)
            );

        operator.rightTrigger()
            .onTrue(new Outtake(elevator, pivot, intake)
            );
            /// make INTAKE ALLWYS RUNNING

        operator.start()
            .onTrue(elevator.resetEncoders());
        operator.back()
            .onTrue(pivot.resetClawEncoder());

        
            // pivot.setPowerC(-operator.getLeftY())
        // operator.rightBumper()
        //     .onTrue(pivot.setPowerC(0.4))
        //     .onFalse(pivot.setPowerC(0));
        
        // operator.leftBumper()
        //     .onTrue(pivot.setPowerC(-0.4))
        //     .onFalse(pivot.setPowerC(0));

        operator.leftTrigger()
        .onTrue(new SequentialCommandGroup(
            new MoveMid(elevator,pivot),
            new WaitCommand(3),
            new DropCone(elevator, pivot, intake)));
    }

    public void configureTestBindings() {
        
    }
    
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
