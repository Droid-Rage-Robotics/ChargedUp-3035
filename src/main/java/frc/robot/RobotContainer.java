package frc.robot;

import frc.robot.commands.Autos;
import frc.robot.commands.arm.*;
import frc.robot.commands.drive.SwerveDriveTeleop;
import frc.robot.subsystem.*;
import frc.robot.subsystem.arm.Arm;
import frc.robot.subsystem.arm.Arm.Position;
import frc.robot.subsystem.arm.elevator.HorizontalElevator;
import frc.robot.subsystem.arm.elevator.VerticalElevator;
import frc.robot.subsystem.arm.elevator.VerticalElevatorSetPower;
import frc.robot.subsystem.arm.pivot.*;
import frc.robot.subsystem.drive.Drive;
import frc.robot.utility.ComplexWidgetBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
    //TODO: Ideas
    // override button for when sensors fail
    // find out how to make a field graph with robot pose on it and preferably a visual for path following
    // custom shuffleboard droid rage theme
    // auto align
    // fix auto balace
    // fix lock wheels
    // limielite
    // detect when intake velocity error drops and light leds
    // Rumble Driver 1 when element in claw
    // see how slew rate limiter affects turning and movement. can it make motors stop faster without enabling brake mode?



    // MAKE ALL OFTHE IS ENABLED IN ONE LOCATION
    private final CommandXboxController driver =
        new CommandXboxController(DroidRageConstants.Gamepad.DRIVER_CONTROLLER_PORT);
    private final CommandXboxController operator =
        new CommandXboxController(DroidRageConstants.Gamepad.OPERATOR_CONTROLLER_PORT);

    private final Drive drive = new Drive();
    private final VerticalElevator verticalElevator = new VerticalElevator();
    // private final VerticalElevatorSetPower verticalElevatorSetPower = new VerticalElevatorSetPower();
    private final HorizontalElevator horizontalElevator = new HorizontalElevator();
    private final PivotAbsolute pivot = new PivotAbsolute();
    private final Intake intake = new Intake();
    private final Arm arm = new Arm(verticalElevator, horizontalElevator, pivot);
    private final Light light = new Light();//Make sure it is after Intake

    

    SendableChooser<CommandBase> autoChooser = new SendableChooser<CommandBase>();

    public RobotContainer() {
        // autoChooser.addOption("PreloadPlusPark", Autos.preloadPlusPark(drive, arm, intake));

        autoChooser.addOption("Forward Test", Autos.forwardTest(drive));
        autoChooser.addOption("Backward Test", Autos.backTest(drive));
        autoChooser.addOption("Spline Test", Autos.splineTest(drive));
        autoChooser.addOption("LineToLinear Test", Autos.lineToLinearTest(drive));
        autoChooser.addOption("StrafeRightTest", Autos.strafeRight(drive));
        autoChooser.addOption("StrafeLeftTest", Autos.strafeLeft(drive));

        autoChooser.addOption("ForwardThenTurnTest", Autos.forwardThenTurnTest(drive));
        // autoChooser.addOption("One: CUbe + drop", Autos.oneToCubeAndToDrop(drive, arm, intake));
        // autoChooser.addOption("three: CUbe + drop", Autos.threeToCubeAndToDrop(drive, arm, intake));
        autoChooser.addOption("Turn Test", Autos.turnTest(drive));
        // autoChooser.addOption("Charge", Autos.charge(drive, arm, intake));
        // autoChooser.addOption("Charge Plus Pickup", Autos.chargePlusPickUp(drive, arm, intake));
        // autoChooser.addOption("dropAndPickupContinnuous", Autos.dropAndPickupContinnuous(drive, arm, intake));
        ComplexWidgetBuilder.create(autoChooser, "Auto Chooser", "Misc")
            .withSize(1, 3);

        // ComplexWidgetBuilder.create(CameraServer.startAutomaticCapture(), "USB Camera Stream", "Misc")
        //     .withSize(5, 5);
    }

    public void configureTeleOpBindings() {
        DriverStation.silenceJoystickConnectionWarning(true);
        // TrackedElement.set();
        light.setDefaultCommand(new IntakeCommand(intake, light, driver));//TODO:Test
        
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


        //     driver.rightTrigger()
        //     .onTrue(intake.run(()->intake.setManualOutput(0.1))) 
        //     .onFalse(intake.run(()->intake.setManualOutput(0)));

        // driver.leftTrigger()
        //     .onTrue(intake.run(()->intake.setManualOutput(-0.1)))
        //     .onFalse(intake.run(()->intake.setManualOutput(0)));

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
                    new ToggleIntake(arm, intake)
                    
                // )
            ); 

        driver.back()
            .onTrue(drive.toggleFieldOriented()
            );
        
        driver.y()
            .onTrue(drive.toggleBreakMode());

        driver.x()
            .onTrue(new ToggleElement());


        /*
         * Operator Controls
         */
        // Trigger pivotManual = 
        pivot.setDefaultCommand(new ManualMotionProfiledPivot(operator::getRightY, pivot)); // This should run the command repeatedly even once its ended if i read correctly
        verticalElevator.setDefaultCommand(new ManualVerticalElevator(operator::getLeftY, verticalElevator));
        horizontalElevator.setDefaultCommand(new ManualHorizontalElevator(operator::getLeftX, horizontalElevator));

        operator.a()
            .whileTrue( // while true to update positions when moving manually
                arm.setPositionCommand(Position.LOW)
            );
        operator.x()
            .whileTrue(
                arm.setPositionCommand(Position.MID)
            );
        operator.y()
            .whileTrue(
                arm.setPositionCommand(Position.HIGH)
            );
        
        operator.povUp()
            .whileTrue(
                arm.setPositionCommand(Position.INTAKE_HIGH_DOUBLE_SUBSTATION)
            );
        operator.povRight()
            .whileTrue(
                arm.setPositionCommand(Position.INTAKE_HIGH_SINGLE_SUBSTATION)
            );
        operator.povLeft()
            .whileTrue(
                arm.setPositionCommand(Position.INTAKE_LOW)
            );
        operator.povDown()
            .whileTrue(
                arm.setPositionCommand(Position.HOLD)
            );
        

        // operator.a()
        //     .onTrue(pivot.runOnce(() -> pivot.setTargetPosition(Math.PI)));

        // operator.start()
        //     .onTrue(Commands.sequence(
        //         verticalElevator.runOnce(verticalElevator::resetEncoder),
        //         horizontalElevator.runOnce(horizontalElevator::resetEncoder)
        //     ));
      
      
        
        // operator.rightTrigger().onTrue(verticalElevatorSetPower.runOnce(()->verticalElevatorSetPower.setPower(1)))
        //                         .onFalse(verticalElevatorSetPower.runOnce(()->verticalElevatorSetPower.stop()));
        // operator.leftTrigger().onTrue(verticalElevatorSetPower.runOnce(()->verticalElevatorSetPower.setPower(-1)))
        //                         .onFalse(verticalElevatorSetPower.runOnce(()->verticalElevatorSetPower.stop()));
        
        // operator.back()
        //     .onTrue(pivot.runOnce(pivot::resetEncoder)); // In absolute mode
    }

    public void configureTestBindings(){}
    
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
