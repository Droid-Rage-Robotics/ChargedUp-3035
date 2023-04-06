package frc.robot;

import frc.robot.commands.LightCommand;
import frc.robot.commands.SuppliedCommand;
import frc.robot.commands.arm.*;
import frc.robot.commands.autoPaths.BumpAutos;
import frc.robot.commands.autoPaths.ChargeAutos;
import frc.robot.commands.autoPaths.FreeAutos;
import frc.robot.commands.autoPaths.TuningAutos;
import frc.robot.commands.drive.SwerveDriveTeleop;
import frc.robot.commands.intakeAndOuttake.TeleopOuttake;
import frc.robot.commands.intakeAndOuttake.ToggleIntake;
import frc.robot.commands.intakeAndOuttake.teleopDrop.DropTeleopCone;
import frc.robot.commands.intakeAndOuttake.teleopDrop.DropTeleopCube;
import frc.robot.subsystem.*;
import frc.robot.subsystem.TrackedElement.Element;
import frc.robot.subsystem.arm.Arm;
import frc.robot.subsystem.arm.Arm.Position;
import frc.robot.subsystem.arm.elevator.HorizontalElevator;
import frc.robot.subsystem.arm.elevator.VerticalElevator;
import frc.robot.subsystem.arm.pivot.*;
import frc.robot.subsystem.drive.Drive;
import frc.robot.utility.ComplexWidgetBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
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
    //Idea: Make a button that automatically aligns to 180 degrees with a press of a button - 
    //      only when held - still leave control of drive to driver except for turning



    //Disable chris's outtake on cone - Important
    //move enabled to one spot
    //Add a velocity offset intake 
    //Make the arm position switch simpler
    
    // MAKE ALL OFTHE IS ENABLED IN ONE LOCATION
    private final CommandXboxController driver =
        new CommandXboxController(DroidRageConstants.Gamepad.DRIVER_CONTROLLER_PORT);
    private final CommandXboxController operator =
        new CommandXboxController(DroidRageConstants.Gamepad.OPERATOR_CONTROLLER_PORT);

    // private final Drive drive = new Drive();
    // private final VerticalElevator verticalElevator = new VerticalElevator();
    // // private final VerticalElevatorSetPower verticalElevatorSetPower = new VerticalElevatorSetPower();
    // private final HorizontalElevator horizontalElevator = new HorizontalElevator();
    // private final PivotAbsolute pivot = new PivotAbsolute();
    // private final Intake intake = new Intake();
    // private final Arm arm = new Arm(verticalElevator, horizontalElevator, pivot, intake);
    // private final Light light = new Light();
    private final Drive drive;
    private final VerticalElevator verticalElevator;
    // private final VerticalElevatorSetPower verticalElevatorSetPower;
    private final HorizontalElevator horizontalElevator;
    private final PivotAbsolute pivot;
    private final Intake intake;
    private final Arm arm;
    private final Light light;

    

    SendableChooser<CommandBase> autoChooser = new SendableChooser<CommandBase>();

    public RobotContainer(Drive drive, VerticalElevator verticalElevator, 
                            HorizontalElevator horizontalElevator, PivotAbsolute pivot, 
                            Intake intake, Arm arm, Light light) {
        this.drive = drive;
        this.verticalElevator = verticalElevator;
        this.horizontalElevator = horizontalElevator;
        this.pivot = pivot;
        this.intake = intake;
        this.arm = arm;
        this.light = light;
        
        autoChooser.addOption("Forward Test", TuningAutos.forwardTest(drive));
        autoChooser.addOption("Backward Test", TuningAutos.backTest(drive));
        autoChooser.addOption("Spline Test", TuningAutos.splineTest(drive));
        autoChooser.addOption("LineToLinear Test", TuningAutos.lineToLinearTest(drive));
        autoChooser.addOption("StrafeRight Test", TuningAutos.strafeRight(drive));
        autoChooser.addOption("StrafeLeft Test", TuningAutos.strafeLeft(drive));
        autoChooser.addOption("Turn Test", TuningAutos.turnTest(drive));
        autoChooser.addOption("ForwardThenTurn Test", TuningAutos.forwardThenTurnTest(drive));

        autoChooser.addOption("1+1 Bump (High_Mid)", BumpAutos.onePlusOneBumpHigh_Mid(drive, arm,intake));
        autoChooser.addOption("1+1 Bump (Mid_Mid)", BumpAutos.onePlusOneBumpMid_Mid(drive, arm,intake));

        autoChooser.addOption("1+1 Free (High_High)", FreeAutos.onePlusOneFreeHigh_High(drive, arm,intake));
        autoChooser.addOption("1+1 Free (High_Mid)", FreeAutos.onePlusOneFreeHigh_Mid(drive, arm,intake));
        autoChooser.addOption("1+1 Free (Mid_Mid)", FreeAutos.onePlusOneFreeMid_Mid(drive, arm,intake));

        autoChooser.addOption("Charge (High)", ChargeAutos.chargeHigh(drive, arm, intake));
        autoChooser.addOption("Charge (Mid)", ChargeAutos.chargeMid(drive, arm, intake));
        autoChooser.addOption("Charge Plus Pickup (High)", ChargeAutos.chargePlusPickUpHigh(drive, arm, intake));
        autoChooser.addOption("Charge Plus Pickup (Mid)", ChargeAutos.chargePlusPickUpMid(drive, arm, intake));
        autoChooser.addOption("Charge Plus Pickup Parts (High)", ChargeAutos.chargePlusPickUpPartsHigh(drive, arm, intake));//ISSUE!!!!


        // autoChooser.addOption("One: CUbe + drop", OldAutos.oneToCubeAndToDrop(drive, arm, intake));
        // autoChooser.addOption("three: CUbe + drop", OldAutos.threeToCubeAndToDrop(drive, arm, intake));
        // autoChooser.addOption("dropAndPickupContinnuous", OldAutos.dropAndPickupContinnuous(drive, arm, intake));
        ComplexWidgetBuilder.create(autoChooser, "Auto Chooser", "Misc")
            .withSize(1, 3);

        // ComplexWidgetBuilder.create(CameraServer.startAutomaticCapture(), "USB Camera Stream", "Misc")
        //     .withSize(5, 5);
    }

    public void configureTeleOpBindings() {
        DriverStation.silenceJoystickConnectionWarning(true);
        // TrackedElement.set();
        light.setDefaultCommand(new LightCommand(intake, light, driver));
        
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
            // .and(null) - allows you to make 
            //one buttn different by having to press 2 buttons
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
            // .onFalse(intake.run(intake::stop)); 
            .onFalse(SuppliedCommand.create(
                () -> switch(TrackedElement.get()) {
                    case CONE -> intake.run(intake::stop);
                    case CUBE -> intake.run(intake::hold);
                }
            ));
            // .onFalse(if(TrackedElement.get()==Element.CUBE)(intake.run(intake::hold)));

        // driver.leftTrigger()
        // // .onTrue(intake.run(intake::outtake))
        // // .onTrue(intake.runOnce(()->intake.setTargetVelocity(Velocity.SHOOT_CONE_HIGH)))
        //     // .onTrue(intake.run(intake::outtake))//TODO:Test!
        //     // .onTrue(intake.run(intake::outtake))
        //     .onTrue(new DropTeleopCube(arm, intake))
        //     .onFalse(intake.run(intake::stop));

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
        driver.y()
            .onTrue(drive.toggleBreakMode());

        // driver.x()
        //     .onTrue(new ToggleElement());

        driver.povUp()  //Have the robot reset in any 90 degree angle - TODO:Test!
            .onTrue(drive.setOffsetCommand(0));
        driver.povRight()
            .onTrue(drive.setOffsetCommand(-90));
        driver.povDown()
            .onTrue(drive.setOffsetCommand(180));
        driver.povLeft()
            .onTrue(drive.setOffsetCommand(90));
        
        driver.back()
            .onTrue(drive.toggleFieldOriented()
            );
        
        


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


        operator.rightTrigger()
            .onTrue(new DropTeleopCone(arm, intake)) 
            .onFalse(intake.run(intake::stop));
        operator.leftTrigger()
            .onTrue(new DropTeleopCube(arm, intake))
            .onFalse(intake.run(intake::stop));


        // operator.rightTrigger()
        //     .whileTrue(
        //         new TeleopOuttake(arm, intake)
        //     ).whileFalse(intake.runOnce(()->intake.stop()));

        // operator.leftTrigger()
        //     .whileTrue(
        //         new DropTeleopCone(arm, intake)
        //     ).whileFalse(intake.runOnce(()->intake.stop()));
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
