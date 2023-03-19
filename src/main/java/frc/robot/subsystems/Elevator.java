package frc.robot.subsystems;

import java.util.function.Supplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.EnumPositions.Position;
import frc.robot.subsystems.EnumPositions.TrackedElement;
import frc.robot.subsystems.EnumPositions.Position.Positions;
import frc.robot.utilities.ComplexWidgetBuilder;
import frc.robot.utilities.MutableBoolean;
import frc.robot.utilities.MutableDouble;
import frc.robot.utilities.SimpleWidgetBuilder;
import frc.robot.utilities.WriteOnlyDouble;
import frc.robot.utilities.WriteOnlyString;

public class Elevator extends SubsystemBase {
    public static class Constants {
        public static final double VERTICAL_GEAR_RATIO = 1 / 1;
        public static final double VERTICAL_GEAR_DIAMETER_INCHES = 1.88;
        public static final double VERTICAL_COUNTS_PER_PULSE = 1; // 2048 bc rev through bore
        public static final double VERTICAL_ROT_TO_INCHES = (VERTICAL_COUNTS_PER_PULSE * VERTICAL_GEAR_RATIO) / (VERTICAL_GEAR_DIAMETER_INCHES * Math.PI);

        public static final double HORIZONTAL_GEAR_RATIO = 1 / 1;
        public static final double HORIZONTAL_GEAR_DIAMETER_INCHES = 1.4;
        public static final double HORIZONTAL_DISTANCE_PER_PULSE = 1; // 1 bc built in encoder
        public static final double HORIZONTAL_ROT_TO_INCHES = (HORIZONTAL_DISTANCE_PER_PULSE * HORIZONTAL_GEAR_RATIO) / (HORIZONTAL_GEAR_DIAMETER_INCHES * Math.PI);
    }

    // public enum ElevatorPosition {//16-17 is MAXXXXXX for vert ; 11 is for horiz
    //     START(0,0),

    //     INTAKELOWCONE(0,0),
    //     INTAKELOWCUBE(0,0),
       
    //     LOWCONE(0, 0),
    //     LOWCUBE(0,0),
        
    //     MIDCONE(13.2,11),
    //     MIDCUBE(13.4,10.4),

    //     AUTOMIDCONE(15.2, 11.5),

    //     HIGHCONE(14.4,11),//Make this Mid Teleop
    //     HIGHCUBE(17,11),

    //     INTAKEHIGH1CONE(14.9,0),
    //     INTAKEHIGH1CUBE(14.9,0),

    //     INTAKEHIGH2CONE(13.5,0),
    //     INTAKEHIGH2CUBE(13.5,0),

    //     HOLD(0,0),
        
    //     ;

    //     private MutableDouble verticalInches;
    //     private MutableDouble horizontalInches;

    //     private ElevatorPosition(double verticalInches, double horizontalInches) {
    //         this.verticalInches = SimpleWidgetBuilder.create(verticalInches, ElevatorPosition.class.getSimpleName()+"/"+name()+"/Vertical (Inches)", Elevator.class.getSimpleName())
    //             .withSize(1, 3)
    //             .buildMutableDouble();
    //         this.horizontalInches = SimpleWidgetBuilder.create(horizontalInches, ElevatorPosition.class.getSimpleName()+"/"+name()+"/Horizontal (Inches)", Elevator.class.getSimpleName())
    //             .withSize(1, 3)
    //             .buildMutableDouble();
    //     }

    //     public static ElevatorPosition get(){
    //         return position;
    //     }
    // }
    
    private final CANSparkMax verticalLeftMotor, verticalRightElevator, horizontalMotor;
    // private final Encoder verticalEncoder;
    private final RelativeEncoder verticalEncoder;
    private final RelativeEncoder horizontalEncoder;
    // private final DutyCycleEncoder verticaAbsEncoder;
    private final PIDController verticalController;
    private final PIDController horizontalController;
    // private static volatile ElevatorPosition position = ElevatorPosition.START;
    private volatile Positions position = Positions.START;
    public boolean isMovingManually;
    private final WriteOnlyString positionWriter = new WriteOnlyString(position.name(), "Elevator Position", Elevator.class.getSimpleName());
    
    private final WriteOnlyDouble verticalEncoderPositionWriter = new WriteOnlyDouble(0, "Vertical Encoder Position (Inches)", Elevator.class.getSimpleName());
    private final WriteOnlyDouble horizontalEncoderPositionWriter = new WriteOnlyDouble(0, "Horizontal Encoder Position (Inches)", Elevator.class.getSimpleName());
    
    // private final WriteOnlyDouble verticalTargetPositionWriter = new WriteOnlyDouble(0, "Vertical Target Position (Meters)", Elevator.class.getSimpleName());
    // private final WriteOnlyDouble horizontalTargetPositionWriter = new WriteOnlyDouble(0, "Horizontal Target Position (Meters)", Elevator.class.getSimpleName());

    private final MutableBoolean isVerticalEnabled = SimpleWidgetBuilder.create(true, "Is Vertical Enabled", Elevator.class.getSimpleName())
        .withWidget(BuiltInWidgets.kToggleSwitch)
        .buildMutableBoolean();

    private final MutableBoolean isHorizontalEnabled = SimpleWidgetBuilder.create(true, "Is Horizontal Enabled", Elevator.class.getSimpleName())
        .withWidget(BuiltInWidgets.kToggleSwitch)
        .buildMutableBoolean();
    
    public Elevator() {
        verticalLeftMotor = new CANSparkMax(16, MotorType.kBrushless);
        verticalRightElevator = new CANSparkMax(15, MotorType.kBrushless);//TODO: Where is it plugged in?
        horizontalMotor = new CANSparkMax(17, MotorType.kBrushless);

        verticalLeftMotor.setIdleMode(IdleMode.kBrake);
        verticalRightElevator.setIdleMode(IdleMode.kBrake);
        horizontalMotor.setIdleMode(IdleMode.kBrake);
        horizontalMotor.setInverted(true);

        verticalRightElevator.follow(verticalLeftMotor, true);
  
        verticalEncoder = verticalLeftMotor.getEncoder();
        verticalEncoder.setPositionConversionFactor(Constants.VERTICAL_ROT_TO_INCHES);

        horizontalEncoder = horizontalMotor.getEncoder();
        horizontalEncoder.setPositionConversionFactor(Constants.HORIZONTAL_ROT_TO_INCHES);

        verticalController = new PIDController(0.2, 0, 0);//0.15
        verticalController.setTolerance(0.1); // inches 
        horizontalController = new PIDController(0.2, 0, 0);//0.1
        horizontalController.setTolerance(0.10); // inches

        ComplexWidgetBuilder.create(verticalController, "Vertical PID Controller", Elevator.class.getSimpleName())
            .withWidget(BuiltInWidgets.kPIDController)
            .withSize(2, 2);
        ComplexWidgetBuilder.create(horizontalController, "Horizontal PID Controller", Elevator.class.getSimpleName())
            .withWidget(BuiltInWidgets.kPIDController)
            .withSize(2, 2);

        setPosition(() -> Positions.START);

        // verticaAbsEncoder = new DutyCycleEncoder(9);/No Absolute
        isMovingManually = false;

        ComplexWidgetBuilder.create(resetElevatorEncoders(), "Reset Elevator Encoders", Elevator.class.getSimpleName());
    }

    private final WriteOnlyDouble horizontalSetPowerWriter = new WriteOnlyDouble(0.0, "horizontal set power", "Elevator");
    private final WriteOnlyDouble verticalSetPowerWriter = new WriteOnlyDouble(0.0, "vertical set power", "Elevator");
    
    private double verticalEncoderOffset = 0;
    private double horizontalEncoderOffset = 0;
    // private final WriteOnlyDouble horizeEcoderReading = new WriteOnlyDouble(0.0, "horiz encoder reading", "Elevator");
    @Override
    public void periodic() {
        setHorizontalPower(horizontalController.calculate(getHorizontalEncoderPosition()));
        setVerticalPower(verticalController.calculate(getVerticalEncoderPosition()));
    }

    @Override
    public void simulationPeriodic() {
        periodic();
    }

    // public ElevatorPosition getTargetPosition() {
    //     return position;
    // }

    public Positions getTargetPosition() {
        return position;
    }

    // public double getTargetVerticalHeight() {
    //     return position.verticalInches.get();
    // }

    // public double getTargetHorizontalDistance() {
    //     return position.horizontalInches.get();
    // }

    public double getTargetVerticalHeight() {
        return Position.getVerticalHeight(position);
    }

    public double getTargetHorizontalDistance() {
        return Position.getHorizonotalDistance(position);
    }

    // public CommandBase setPosition(Supplier<ElevatorPosition> position) {
    //     return runOnce(() -> {
    //         Elevator.position = position.get();

    //         verticalController.setSetpoint(getTargetVerticalHeight());
    //         horizontalController.setSetpoint(getTargetHorizontalDistance());

    //         positionWriter.set(getTargetPosition().name());
    //     });
    // }

    public CommandBase setPosition(Supplier<Positions> position) {
        return runOnce(() -> {
            this.position = position.get();
            Position.set(this.position);
            verticalController.setSetpoint(getTargetVerticalHeight());
            horizontalController.setSetpoint(getTargetHorizontalDistance());
        });
    }




    // public CommandBase runSetVerticalPosition(double verticalPosition) {
    //     return runOnce(() -> position.verticalInches.set(verticalPosition));
    // }
    // public CommandBase runSetHorizontalPosition(double horizontalPosition) {
    //     return runOnce(() -> position.horizontalInches.set(horizontalPosition));
    // }
    
    /*public CommandBase setPosition(double vertPosition, double horizPosition) {
        return runOnce(() -> {
            position.verticalInches.set(vertPosition);
            position.horizontalInches.set(horizPosition);

            verticalController.setSetpoint(vertPosition);
            horizontalController.setSetpoint(horizPosition);

            positionWriter.set(getTargetPosition().name()+" (Modified)");
        });
    }*/

    public CommandBase moveHold() {
        return setPosition(() -> Positions.HOLD);
    }

    public CommandBase moveIntakeLow() {
        return setPosition(
            () -> switch(TrackedElement.get()) {
                case CONE -> Positions.INTAKELOWCONE;
                case CUBE -> Positions.INTAKELOWCUBE;
                case NONE -> Positions.INTAKELOWCUBE;
            }
        );
    }
    public CommandBase moveIntake1High() {
        return setPosition(
            () ->switch(TrackedElement.get()) {
                case CONE -> Positions.INTAKEHIGH1CONE;
                case CUBE -> Positions.INTAKEHIGH1CUBE;
                case NONE -> Positions.INTAKEHIGH1CUBE;
            }
        );
    }

    public CommandBase moveIntake2High() {
        return setPosition(
            () ->switch(TrackedElement.get()) {
                case CONE -> Positions.INTAKEHIGH2CONE;
                case CUBE -> Positions.INTAKEHIGH2CUBE;
                case NONE -> Positions.INTAKEHIGH2CUBE;
            }
        );
    }

    public CommandBase moveLow() {
        return setPosition(
            () ->switch(TrackedElement.get()) {
                case CONE -> Positions.LOWCONE;
                case CUBE -> Positions.LOWCUBE;
                case NONE -> Positions.LOWCUBE;
            }
        );
    }

    public CommandBase moveMid() {
        return setPosition(
            () ->switch(TrackedElement.get()) {
                case CONE -> Positions.MIDCONE;
                case CUBE -> Positions.MIDCUBE;
                case NONE -> Positions.MIDCUBE;
            }
        );
    }

    public CommandBase moveAutoMid() {
        return setPosition(() ->Positions.AUTOMIDCONE);
    }

    public CommandBase moveHigh() {
        // if(isMovingManually){//TODO:Test
        //     changePosition();
        // }
        return setPosition(
            () ->switch(TrackedElement.get()) {
                case CONE -> Positions.HIGHCONE;
                case CUBE -> Positions.HIGHCUBE;
                case NONE -> Positions.HIGHCUBE;
            }
        );
    }

    // public void changePosition() {
    //     position.verticalInches.set(getTargetVerticalHeight());
    //     position.horizontalInches.set(getTargetHorizontalDistance());
    // }

    private void setHorizontalPower(double power) {
        horizontalSetPowerWriter.set(power);
        if (!isHorizontalEnabled.get()) return;
        horizontalMotor.set(power);
    }

    private void setVerticalPower(double power) {
        verticalSetPowerWriter.set(power);
        if (!isVerticalEnabled.get()) return;
        verticalLeftMotor.set(power);
    }
    public CommandBase dropVerticalElevator(){
        return runOnce(() -> verticalController.setSetpoint(getTargetVerticalHeight()-7));
    }

    public CommandBase moveInHorizontalElevator(){
        return runOnce(() -> verticalController.setSetpoint(getTargetHorizontalDistance()-2));
    }

    public CommandBase resetElevatorEncoders() {
        return runOnce(() -> {
            verticalEncoderOffset = -getVerticalEncoderPosition() - verticalEncoderOffset;
            horizontalEncoderOffset = -getHorizontalEncoderPosition() - horizontalEncoderOffset;
            SimpleWidgetBuilder.create(true, "Elevator Encoders were reset", Elevator.class.getSimpleName());
        });
    }

    public double getVerticalEncoderPosition() {
        double verticalEncoderPosition = verticalEncoder.getPosition() + verticalEncoderOffset;
        verticalEncoderPositionWriter.set(verticalEncoderPosition);
        return verticalEncoderPosition;
    }
    public double getHorizontalEncoderPosition() {
        double encoderPosition = horizontalEncoder.getPosition();
        horizontalEncoderPositionWriter.set(encoderPosition);
        return encoderPosition;
    }

    public double getVerticalTargetPosition() {
        return verticalController.getSetpoint();
    }

    public double getHorizonotalTargetPosition() {
        return horizontalController.getSetpoint();
    }

    public void setTargetPositionsManually(double x, double y) {
        verticalController.setSetpoint(getVerticalTargetPosition() + (y*0.1));
        // horizontalController.setSetpoint(getHorizontalEncoderPosition() + (x*0.1));
    }

    // public static Position get(){
    //     return position;
    // }
}  
