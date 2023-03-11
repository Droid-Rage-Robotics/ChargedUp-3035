package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

    public enum ElevatorPosition {//16-17 is MAXXXXXX for vert ; 11 is for horiz
        START(0,0),

        INTAKELOWCONE(0,0),
        INTAKELOWCUBE(0,0),
       
        LOWCONE(0, 0),
        LOWCUBE(0,0),
        
        MIDCONE(13.2,11),
        MIDCUBE(13.4,10.4),

        AUTOMIDCONE(15.5, 11.5),

        HIGHCONE(14.4,11),//Make this Mid Teleop
        HIGHCUBE(16.5,11),

        INTAKEHIGH1CONE(14.9,0),
        INTAKEHIGH1CUBE(14.9,0),

        INTAKEHIGH2CONE(13,0),
        INTAKEHIGH2CUBE(14.9,0),

        HOLD(0,0),
        
        ;

        private MutableDouble verticalInches;
        private MutableDouble horizontalInches;

        private ElevatorPosition(double verticalInches, double horizontalInches) {
            this.verticalInches = new SimpleWidgetBuilder<Double>(verticalInches, ElevatorPosition.class.getSimpleName()+"/"+name()+"/Vertical (Inches)", Elevator.class.getSimpleName())
                .withSize(1, 3)
                .buildMutableDouble();
            this.horizontalInches = new SimpleWidgetBuilder<Double>(horizontalInches, ElevatorPosition.class.getSimpleName()+"/"+name()+"/Horizontal (Inches)", Elevator.class.getSimpleName())
                .withSize(1, 3)
                .buildMutableDouble();
        }

        public static ElevatorPosition get(){
            return position;
        }
    }
    
    private final CANSparkMax verticalLeftMotor, verticalRightElevator, horizontalMotor;
    // private final Encoder verticalEncoder;
    private final RelativeEncoder verticalEncoder;
    private final RelativeEncoder horizontalEncoder;
    // private final DutyCycleEncoder verticaAbsEncoder;
    private final PIDController verticalController;
    private final PIDController horizontalController;
    private volatile static ElevatorPosition position = ElevatorPosition.START;
    public boolean isMovingManually;
    private final WriteOnlyString positionWriter = new WriteOnlyString(position.name(), "Elevator Position", Elevator.class.getSimpleName());
    
    private final WriteOnlyDouble verticalEncoderPositionWriter = new WriteOnlyDouble(0, "Vertical Encoder Position (Inches)", Elevator.class.getSimpleName());
    private final WriteOnlyDouble horizontalEncoderPositionWriter = new WriteOnlyDouble(0, "Horizontal Encoder Position (Inches)", Elevator.class.getSimpleName());
    
    // private final WriteOnlyDouble verticalTargetPositionWriter = new WriteOnlyDouble(0, "Vertical Target Position (Meters)", Elevator.class.getSimpleName());
    // private final WriteOnlyDouble horizontalTargetPositionWriter = new WriteOnlyDouble(0, "Horizontal Target Position (Meters)", Elevator.class.getSimpleName());

    private final MutableBoolean isVerticalEnabled = new SimpleWidgetBuilder<Boolean>(true, "Is Vertical Enabled", Elevator.class.getSimpleName())
        .withWidget(BuiltInWidgets.kToggleSwitch)
        .buildMutableBoolean();

    private final MutableBoolean isHorizontalEnabled = new SimpleWidgetBuilder<Boolean>(true, "Is Horizontal Enabled", Elevator.class.getSimpleName())
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

        verticalController = new PIDController(0.15, 0, 0);
        verticalController.setTolerance(0.1); // inches //TODO:How much tolerence?
        horizontalController = new PIDController(0.1, 0, 0);
        horizontalController.setTolerance(0.10); // inches

        new ComplexWidgetBuilder(verticalController, "Vertical PID Controller", Elevator.class.getSimpleName())
            .withWidget(BuiltInWidgets.kPIDController)
            .withSize(2, 2);
        new ComplexWidgetBuilder(horizontalController, "Horizontal PID Controller", Elevator.class.getSimpleName())
            .withWidget(BuiltInWidgets.kPIDController)
            .withSize(2, 2);

        setPosition(ElevatorPosition.START);

        // verticaAbsEncoder = new DutyCycleEncoder(9);/No Absolute
        isMovingManually = false;

        new ComplexWidgetBuilder(resetElevatorEncoders(), "Reset Elevator Encoders", Elevator.class.getSimpleName());
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

    public ElevatorPosition getTargetPosition() {
        return position;
    }

    public double getTargetVerticalHeight() {
        return position.verticalInches.get();
    }

    public double getTargetHorizontalDistance() {
        return position.horizontalInches.get();
    }

    public CommandBase setPosition(ElevatorPosition position) {
        return runOnce(() -> {
            this.position = position;

            verticalController.setSetpoint(getTargetVerticalHeight());
            horizontalController.setSetpoint(getTargetHorizontalDistance());

            positionWriter.set(getTargetPosition().name());
        });
    }
    // public CommandBase runSetVerticalPosition(double verticalPosition) {
    //     return runOnce(() -> position.verticalInches.set(verticalPosition));
    // }
    // public CommandBase runSetHorizontalPosition(double horizontalPosition) {
    //     return runOnce(() -> position.horizontalInches.set(horizontalPosition));
    // }
    public CommandBase setPosition(double vertPosition, double horizPosition) {
        return runOnce(() -> {
            position.verticalInches.set(vertPosition);
            position.horizontalInches.set(horizPosition);

            verticalController.setSetpoint(vertPosition);
            horizontalController.setSetpoint(horizPosition);

            positionWriter.set(getTargetPosition().name()+" (Modified)");
        });
    }

    public CommandBase moveHold() {
        return setPosition(ElevatorPosition.HOLD);
    }

    public CommandBase moveIntakeLow() {
        return setPosition(
            switch(TrackedElement.get()) {
                case CONE -> ElevatorPosition.INTAKELOWCONE;
                case CUBE -> ElevatorPosition.INTAKELOWCUBE;
                case NONE -> ElevatorPosition.INTAKELOWCUBE;
            }
        );
    }
    public CommandBase moveIntake1High() {
        return setPosition(
            switch(TrackedElement.get()) {
                case CONE -> ElevatorPosition.INTAKEHIGH1CONE;
                case CUBE -> ElevatorPosition.INTAKEHIGH1CUBE;
                case NONE -> ElevatorPosition.INTAKEHIGH1CUBE;
            }
        );
    }

    public CommandBase moveIntake2High() {
        return setPosition(
            switch(TrackedElement.get()) {
                case CONE -> ElevatorPosition.INTAKEHIGH2CONE;
                case CUBE -> ElevatorPosition.INTAKEHIGH2CUBE;
                case NONE -> ElevatorPosition.INTAKEHIGH2CUBE;
            }
        );
    }

    public CommandBase moveLow() {
        return setPosition(
            switch(TrackedElement.get()) {
                case CONE -> ElevatorPosition.LOWCONE;
                case CUBE -> ElevatorPosition.LOWCUBE;
                case NONE -> ElevatorPosition.LOWCUBE;
            }
        );
    }

    public CommandBase moveMid() {
        return setPosition(
            switch(TrackedElement.get()) {
                case CONE -> ElevatorPosition.MIDCONE;
                case CUBE -> ElevatorPosition.MIDCUBE;
                case NONE -> ElevatorPosition.MIDCUBE;
            }
        );
    }

    public CommandBase moveAutoMid() {
        return setPosition(ElevatorPosition.AUTOMIDCONE);
    }

    public CommandBase moveHigh() {
        if(isMovingManually){//TODO:Test
            changePosition();
        }
        return setPosition(
            switch(TrackedElement.get()) {
                case CONE -> ElevatorPosition.HIGHCONE;
                case CUBE -> ElevatorPosition.HIGHCUBE;
                case NONE -> ElevatorPosition.HIGHCUBE;
            }
        );
    }

    public void changePosition() {
        position.verticalInches.set(getTargetVerticalHeight());
        position.horizontalInches.set(getTargetHorizontalDistance());
    }

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
            new SimpleWidgetBuilder<Boolean>(true, "Elevator Encoders were reset", Elevator.class.getSimpleName());
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
