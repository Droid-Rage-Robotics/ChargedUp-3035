package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
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
        public static final double VERTICAL_DISTANCE_PER_PULSE = 2048; // 2048 bc rev through bore
        public static final double VERTICAL_ROT_TO_INCHES = VERTICAL_DISTANCE_PER_PULSE * VERTICAL_GEAR_RATIO / (VERTICAL_GEAR_DIAMETER_INCHES * Math.PI);

        public static final double HORIZONTAL_GEAR_RATIO = 1 / 1;
        public static final double HORIZONTAL_GEAR_DIAMETER_INCHES = 1.4;
        public static final double HORIZONTAL_DISTANCE_PER_PULSE = 1; // 1 bc built in encoder
        public static final double HORIZONTAL_ROT_TO_INCHES = HORIZONTAL_DISTANCE_PER_PULSE * HORIZONTAL_GEAR_RATIO / (HORIZONTAL_GEAR_DIAMETER_INCHES * Math.PI);
    }

    private enum Position {
        START(0,0),

        INTAKELOW(0,3),
       
        LOWCONE(0, 1),
        LOWCUBE(0,1),
        
        MIDCONE(0,2),
        MIDCUBE(0,2),
        
        HIGHCONE(0,3),
        HIGHCUBE(0,3),

        INTAKEHIGH(0,0),

        HOLD(0,0),
        
        ;

        private MutableDouble verticalInches;
        private MutableDouble horizontalInches;

        private Position(double verticalInches, double horizontalInches) {
            this.verticalInches = new MutableDouble(verticalInches, Position.class.getSimpleName()+"/"+name()+"/Vertical (Inches)", Elevator.class.getSimpleName());
            this.horizontalInches = new MutableDouble(horizontalInches, Position.class.getSimpleName()+"/"+name()+"/Horizontal (Inches)", Elevator.class.getSimpleName());
        }
    }
    
    private final CANSparkMax verticalLeftMotor, verticalRightElevator, horizontalMotor;
    private final Encoder verticalEncoder;
    private final RelativeEncoder horizontalEncoder;
    // private final DutyCycleEncoder verticaAbsEncoder;
    private final PIDController verticalController;
    private final PIDController horizontalController;
    private volatile Position position = Position.START;
    public boolean isMovingManually;
    private final WriteOnlyString positionWriter = new WriteOnlyString(position.name(), "Elevator Position", Elevator.class.getSimpleName());
    
    private final WriteOnlyDouble verticalEncoderPositionWriter = new WriteOnlyDouble(0, "Vertical Encoder Position (Meters)", Elevator.class.getSimpleName());
    private final WriteOnlyDouble horizontalEncoderPositionWriter = new WriteOnlyDouble(0, "Horizontal Encoder Position (Meters)", Elevator.class.getSimpleName());
    
    // private final WriteOnlyDouble verticalTargetPositionWriter = new WriteOnlyDouble(0, "Vertical Target Position (Meters)", Elevator.class.getSimpleName());
    // private final WriteOnlyDouble horizontalTargetPositionWriter = new WriteOnlyDouble(0, "Horizontal Target Position (Meters)", Elevator.class.getSimpleName());

    private final MutableBoolean isVerticalEnabled = new SimpleWidgetBuilder<Boolean>(false, "Is Vertical Enabled", Elevator.class.getSimpleName())
        .withWidget(BuiltInWidgets.kToggleSwitch)
        .buildMutableBoolean();

    private final MutableBoolean isHorizontalEnabled = new SimpleWidgetBuilder<Boolean>(false, "Is Horizontal Enabled", Elevator.class.getSimpleName())
        .withWidget(BuiltInWidgets.kToggleSwitch)
        .buildMutableBoolean();
    
    public Elevator() {
        verticalLeftMotor = new CANSparkMax(16, MotorType.kBrushless);
        verticalRightElevator = new CANSparkMax(15, MotorType.kBrushless);//TODO: Where is it plugged in?
        horizontalMotor = new CANSparkMax(17, MotorType.kBrushless);

        verticalLeftMotor.setIdleMode(IdleMode.kCoast);
        verticalRightElevator.setIdleMode(IdleMode.kCoast);
        horizontalMotor.setIdleMode(IdleMode.kCoast);
        horizontalMotor.setInverted(true);

        verticalRightElevator.follow(verticalLeftMotor, true);
  
        verticalEncoder = new Encoder(9, 8);
        verticalEncoder.setDistancePerPulse(Constants.VERTICAL_ROT_TO_INCHES);

        horizontalEncoder = horizontalMotor.getEncoder();
        horizontalEncoder.setPositionConversionFactor(Constants.HORIZONTAL_ROT_TO_INCHES);

        verticalController = new PIDController(0.01, 0, 0);
        verticalController.setTolerance(0.1); // inches //TODO:How much tolerence?
        horizontalController = new PIDController(0.05, 0, 0);
        horizontalController.setTolerance(0.10); // inches

        new ComplexWidgetBuilder(verticalController, "Vertical PID Controller", Elevator.class.getSimpleName())
            .withWidget(BuiltInWidgets.kPIDController)
            .withSize(2, 2);
        new ComplexWidgetBuilder(horizontalController, "Horizontal PID Controller", Elevator.class.getSimpleName())
            .withWidget(BuiltInWidgets.kPIDController)
            .withSize(2, 2);

        setPosition(Position.START);

        // verticaAbsEncoder = new DutyCycleEncoder(9);/No Absolute
        isMovingManually = false;
    }

    private final WriteOnlyDouble horizontalSetPowerWriter = new WriteOnlyDouble(0.0, "horizontal set power", "Elevator");
    private final WriteOnlyDouble verticalSetPowerWriter = new WriteOnlyDouble(0.0, "vertical set power", "Elevator");
    
    // private final WriteOnlyDouble horizeEcoderReading = new WriteOnlyDouble(0.0, "horiz encoder reading", "Elevator");
    @Override
    public void periodic() {
        setHorizontalPower(horizontalController.calculate(horizontalEncoder.getPosition()));
        horizontalSetPowerWriter.set(horizontalController.calculate(horizontalEncoder.getPosition()));
        horizontalEncoderPositionWriter.set(horizontalEncoder.getPosition());


        setVerticalPower(verticalController.calculate(verticalEncoder.getDistance()));
        verticalSetPowerWriter.set(verticalController.calculate(verticalEncoder.getDistance()));

        
        verticalEncoderPositionWriter.set(verticalEncoder.getDistance());
    }

    @Override
    public void simulationPeriodic() {
        periodic();
    }

    public Position getTargetPosition() {
        return position;
    }

    public double getTargetVerticalHeight() {
        return position.verticalInches.get();
    }

    public double getTargetHorizontalDistance() {
        return position.horizontalInches.get();
    }

    public CommandBase setPosition(Position position) {
        return runOnce(() -> {
            this.position = position;

            verticalController.setSetpoint(getTargetVerticalHeight());
            horizontalController.setSetpoint(getTargetHorizontalDistance());

            positionWriter.set(getTargetPosition().name());
        });
    }
    public CommandBase setPosition(double vertPosition, double horizPosition) {
        return runOnce(() -> {
            verticalController.setSetpoint(vertPosition);
            horizontalController.setSetpoint(horizPosition);

            positionWriter.set(getTargetPosition().name()+" (Modified)");
        });
    }

    public CommandBase moveHold() {
        return setPosition(Position.HOLD);
    }

    public CommandBase moveIntakeLow() {
        return setPosition(Position.INTAKELOW);
    }
    public CommandBase moveIntakeHigh() {
        return setPosition(Position.INTAKEHIGH);
    }

    public CommandBase moveLow() {
        return setPosition(
            switch(TrackedElement.get()) {
                case CONE -> Position.LOWCONE;
                case CUBE -> Position.LOWCUBE;
                case NONE -> Position.LOWCUBE;
            }
        );
    }

    public CommandBase moveMid() {
        return setPosition(
            switch(TrackedElement.get()) {
                case CONE -> Position.MIDCONE;
                case CUBE -> Position.MIDCUBE;
                case NONE -> Position.MIDCUBE;
            }
        );
    }

    public CommandBase moveHigh() {
        if(isMovingManually){//TODO:Test
            changePosition();
        }
        return setPosition(
            switch(TrackedElement.get()) {
                case CONE -> Position.HIGHCONE;
                case CUBE -> Position.HIGHCUBE;
                case NONE -> Position.HIGHCUBE;
            }
        );
    }

    public void changePosition() {
        position.verticalInches.set(getTargetVerticalHeight());
        position.horizontalInches.set(getTargetHorizontalDistance());
    }

    private void setHorizontalPower(double power) {
        if (!isHorizontalEnabled.get()) return;
        horizontalMotor.set(power);
    }

    private void setVerticalPower(double power) {
        if (!isVerticalEnabled.get()) return;
        verticalLeftMotor.set(power);
    }
    public CommandBase dropVerticalElevator(){
        return runOnce(() -> verticalController.setSetpoint(getTargetVerticalHeight()-2));
    }

    public CommandBase moveInHorizontalElevator(){
        return runOnce(() -> verticalController.setSetpoint(getTargetHorizontalDistance()-2));
    }
}  
