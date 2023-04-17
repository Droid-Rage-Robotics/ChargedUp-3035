package frc.robot;

import frc.robot.commands.LightCommand;
import frc.robot.commands.arm.*;
import frc.robot.commands.autoPaths.BumpAutos;
import frc.robot.commands.autoPaths.ChargeAutos;
import frc.robot.commands.autoPaths.FreeAutos;
import frc.robot.commands.autoPaths.TuningAutos;
import frc.robot.commands.drive.SwerveDriveTeleop;
import frc.robot.commands.intakeAndOuttake.teleopDrop.DropTeleopCone;
import frc.robot.commands.intakeAndOuttake.teleopDrop.DropTeleopCube;
import frc.robot.subsystem.*;
import frc.robot.subsystem.arm.Arm;
import frc.robot.subsystem.arm.Arm.Position;
import frc.robot.subsystem.arm.elevator.HorizontalElevator;
import frc.robot.subsystem.arm.elevator.VerticalElevator;
import frc.robot.subsystem.arm.pivot.*;
import frc.robot.subsystem.drive.Drive;
import frc.robot.utility.ComplexWidgetBuilder;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
    //TODO: Ideas
    //Logging Battery usage
    //Logging Robot Positioning
    // override button for when sensors fail
    // custom shuffleboard droid rage theme
    // auto align
    // fix auto balance
    // fix lock wheels
    // limelight
    // see how slew rate limiter affects turning and movement. can it make motors stop faster without enabling brake mode?
    //Idea: Make a button that automatically aligns to 180 degrees with a press of a button - 
    //      only when held - still leave control of drive to driver except for turning



    //Add a velocity offset intake 

    //save time on bump don't worry about free
    //Add lights to have the robot tell us any errors with can, etc.
    
    private final CommandXboxController driver =
        new CommandXboxController(DroidRageConstants.Gamepad.DRIVER_CONTROLLER_PORT);
    private final CommandXboxController operator =
        new CommandXboxController(DroidRageConstants.Gamepad.OPERATOR_CONTROLLER_PORT);

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
        autoChooser.addOption("1+1 Free (Mid_Mid)Pear", FreeAutos.onePlusOneFreeMid_MidPear(drive, arm,intake));
        autoChooser.addOption("1+1 Free (Mid_High)", FreeAutos.onePlusOneFreeMid_High(drive, arm,intake));

        autoChooser.addOption("Charge (High)", ChargeAutos.chargeHigh(drive, arm, intake, light));
        autoChooser.setDefaultOption("Charge (Mid)", ChargeAutos.chargeMid(drive, arm, intake, light));
        autoChooser.addOption("Charge Plus Pickup (High)", ChargeAutos.chargePlusPickUpHigh(drive, arm, intake, light));
        autoChooser.addOption("Charge Plus Pickup (Mid)", ChargeAutos.chargePlusPickUpMid(drive, arm, intake, light));
        autoChooser.addOption("Charge Taxi 180 (Mid)", ChargeAutos.chargeMidTaxi180(drive, arm, intake, light));
        autoChooser.addOption("Charge Taxi 90 (Mid)", ChargeAutos.chargeMidTaxi90(drive, arm, intake, light));
        
        // autoChooser.addOption("Charge Plus Pickup Parts (High)", OldAutos.chargePlusPickUpPartsHigh(drive, arm, intake));//Doesn't Work - Amost Tipped bot in practice
        // autoChooser.addOption("One: CUbe + drop", OldAutos.oneToCubeAndToDrop(drive, arm, intake));
        // autoChooser.addOption("three: CUbe + drop", OldAutos.threeToCubeAndToDrop(drive, arm, intake));
        // autoChooser.addOption("dropAndPickupContinnuous", OldAutos.dropAndPickupContinnuous(drive, arm, intake));
        ComplexWidgetBuilder.create(autoChooser, "Auto Chooser", "Misc")
            .withSize(1, 3);

        ComplexWidgetBuilder.create(CameraServer.startAutomaticCapture(), "USB Camera Stream", "Misc")
        
            .withSize(5, 5);
    }

    public void configureTeleOpBindings() {
        DriverStation.silenceJoystickConnectionWarning(true);
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
            .whileTrue(drive.setSlowSpeed())
            .onFalse(drive.setNormalSpeed());
        driver.rightTrigger()
            .whileTrue(intake.run(intake::intake))
            // .onTrue(arm.setPositionCommand(Position.INTAKE_LOW))

            .onFalse(intake.run(intake::stop));
            // .onFalse(arm.setPositionCommand(Position.HOLD));
        driver.leftTrigger()
            .whileTrue(intake.run(intake::intake))
            .onTrue(arm.setPositionCommand(Position.INTAKE_LOW_DROPPED))

            .onFalse(intake.run(intake::stop))
            .onFalse(arm.setPositionCommand(Position.HOLD));

        driver.b()
            .onTrue(intake.runOnce(()->intake.open(true))); 
        driver.x()
            .onTrue(intake.runOnce(()->intake.close(true)));

        driver.povUp()  //Have the robot reset in any 90 degree angle - TODO:Test!
            .onTrue(drive.setOffsetCommand(0));
        driver.povRight()
            .onTrue(drive.setOffsetCommand(-90));
        driver.povDown()
            .onTrue(drive.setOffsetCommand(180));
        driver.povLeft()
            .onTrue(drive.setOffsetCommand(90));
        
        driver.back()
            .onTrue(drive.toggleFieldOriented());
        
        


        /*
         * Operator Controls
         */
        pivot.setDefaultCommand(new ManualMotionProfiledPivot(operator::getRightY, pivot)); // This should run the command repeatedly even once its ended if i read correctly
        verticalElevator.setDefaultCommand(new ManualVerticalElevator(operator::getLeftY, verticalElevator));
        horizontalElevator.setDefaultCommand(new ManualHorizontalElevator(operator::getLeftX, horizontalElevator));

        operator.a()
            .onTrue( // while true to update positions when moving manually
                arm.setPositionCommand(Position.LOW)
            );
        operator.x()
            .onTrue(arm.setPositionCommand(Position.MID));
        operator.y()
            .onTrue(arm.setPositionCommand(Position.HIGH));
        
        operator.povUp()
            .onTrue(arm.setPositionCommand(Position.INTAKE_HIGH_DOUBLE_SUBSTATION));
        operator.povRight()
            .onTrue(arm.setPositionCommand(Position.INTAKE_HIGH_SINGLE_SUBSTATION));
        operator.povLeft()
            .onTrue(arm.setPositionCommand(Position.INTAKE_LOW));
        operator.povDown()
            .onTrue(arm.setPositionCommand(Position.HOLD));


        operator.rightTrigger()
            .onTrue(new DropTeleopCone(arm, intake)) 
            .onFalse(intake.run(intake::stop))
            .onFalse(arm.setPositionCommand(Position.HOLD));
        operator.leftTrigger()
            .onTrue(new DropTeleopCube(arm, intake))
            .onFalse(intake.run(intake::stop))
            .onFalse(arm.setPositionCommand(Position.HOLD));
    }

    /*************************************/

    public void configureTeleOpDriverOnlyBindings() {
        DriverStation.silenceJoystickConnectionWarning(true);
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
            .whileTrue(drive.setSlowSpeed())
            .onFalse(drive.setNormalSpeed());
        
        driver.rightTrigger().and(driver.leftBumper().negate())
            .whileTrue(intake.run(intake::intake))
            .onTrue(arm.setPositionCommand(Position.INTAKE_LOW))

            .onFalse(intake.run(intake::stop))
            .onFalse(arm.setPositionCommand(Position.HOLD));
        driver.rightTrigger().and(driver.leftBumper())
            .whileTrue(intake.run(intake::intake))
            .onTrue(arm.setPositionCommand(Position.INTAKE_LOW_DROPPED))

            .onFalse(intake.run(intake::stop))
            .onFalse(arm.setPositionCommand(Position.HOLD)); 

        driver.leftTrigger().and(driver.leftBumper())
            .onTrue(new DropTeleopCone(arm, intake)) 
            .onFalse(intake.run(intake::stop))
            .onFalse(arm.setPositionCommand(Position.HOLD));
        driver.leftTrigger().and(driver.leftBumper().negate())
            .onTrue(new DropTeleopCube(arm, intake))
            .onFalse(intake.run(intake::stop))
            .onFalse(arm.setPositionCommand(Position.HOLD));

        driver.a()
            .onTrue(arm.setPositionCommand(Position.LOW));//Should be on True imo
        driver.x()
            .onTrue(arm.setPositionCommand(Position.MID));
        driver.y()
            .onTrue(arm.setPositionCommand(Position.HIGH));
        
        driver.b().and(driver.leftBumper().negate())
            .onTrue(intake.runOnce(()->intake.open(true))); 
        driver.b().and(driver.leftBumper())
            .onTrue(intake.runOnce(()->intake.close(true)));


        driver.povUp().and(driver.leftBumper().negate())  //Have the robot reset in any 90 degree angle - TODO:Test!
            .onTrue(drive.setOffsetCommand(0));
        driver.povUp().and(driver.leftBumper())
            .onTrue(drive.setOffsetCommand(180));
        
        driver.povRight().and(driver.leftBumper().negate())
            .onTrue(drive.setOffsetCommand(-90));
        driver.povRight().and(driver.leftBumper())
            .onTrue(drive.setOffsetCommand(90));

        driver.povLeft().and(driver.leftBumper().negate())
            .onTrue(arm.setPositionCommand(Position.INTAKE_HIGH_DOUBLE_SUBSTATION));
        driver.povLeft().and(driver.leftBumper())
            .onTrue(arm.setPositionCommand(Position.INTAKE_HIGH_SINGLE_SUBSTATION));

        driver.povDown().and(driver.leftBumper())
            .onTrue(arm.setPositionCommand(Position.INTAKE_LOW));
        driver.povDown().and(driver.leftBumper().negate())
            .onTrue(arm.setPositionCommand(Position.HOLD));

        driver.back().onTrue(drive.toggleFieldOriented());
        
        


        /*
         * Operator Controls
         */
        pivot.setDefaultCommand(new ManualMotionProfiledPivot(operator::getRightY, pivot)); // This should run the command repeatedly even once its ended if i read correctly
        verticalElevator.setDefaultCommand(new ManualVerticalElevator(operator::getLeftY, verticalElevator));
        horizontalElevator.setDefaultCommand(new ManualHorizontalElevator(operator::getLeftX, horizontalElevator));
    }

    public void configureTestBindings(){}
    
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
