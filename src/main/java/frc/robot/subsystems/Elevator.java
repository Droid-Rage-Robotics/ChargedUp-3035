package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.ComplexWidgetBuilder;
import frc.robot.utilities.MutableDouble;
import frc.robot.utilities.WriteOnlyDouble;
import frc.robot.utilities.WriteOnlyString;

public class Elevator extends SubsystemBase {
    public static class Constants {
        public static final double VERTICAL_GEAR_RATIO = 1 / 1;
        public static final double VERTICAL_GEAR_DIAMETER_INCHES = 1.88;
        public static final double VERTICAL_ROT_TO_INCHES = 1/ (VERTICAL_GEAR_RATIO * Math.PI * VERTICAL_GEAR_DIAMETER_INCHES);

        public static final double HORIZONTAL_GEAR_RATIO = 1 / 1;
        public static final double HORIZONTAL_GEAR_DIAMETER_INCHES = 1.4;
        public static final double HORIZONTAL_ROT_TO_INCHES = 1/ (HORIZONTAL_GEAR_RATIO * Math.PI * HORIZONTAL_GEAR_DIAMETER_INCHES);
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

        INTAKEHIGH(0,0)
        
        ;

        private MutableDouble verticalInches;
        private MutableDouble horizontalInches;

        private Position(double verticalInches, double horizontalInches) {
            this.verticalInches = new MutableDouble(verticalInches, Position.class.getSimpleName()+"/"+name()+"/Vertical (Inches)", Elevator.class.getSimpleName());
            this.horizontalInches = new MutableDouble(horizontalInches, Position.class.getSimpleName()+"/"+name()+"/Horizontal (Inches)", Elevator.class.getSimpleName());
        }
    }
    
    private final CANSparkMax verticalLeftMotor, verticalRightElevator, horizontalMotor;
    private final RelativeEncoder verticalEncoder, horizontalEncoder;
    private final DutyCycleEncoder verticaAbsEncoder;
    private final PIDController verticalController;
    private final PIDController horizontalController;
    private volatile Position position = Position.START;
    public boolean isMovingManually;
    private final WriteOnlyString positionWriter = new WriteOnlyString(position.name(), "Elevator Position", Elevator.class.getSimpleName());
    
    private final WriteOnlyDouble verticalEncoderPositionWriter = new WriteOnlyDouble(0, "Vertical Encoder Position (Meters)", Elevator.class.getSimpleName());
    private final WriteOnlyDouble horizontalEncoderPositionWriter = new WriteOnlyDouble(0, "Horizontal Encoder Position (Meters)", Elevator.class.getSimpleName());
    
    // private final WriteOnlyDouble verticalTargetPositionWriter = new WriteOnlyDouble(0, "Vertical Target Position (Meters)", Elevator.class.getSimpleName());
    // private final WriteOnlyDouble horizontalTargetPositionWriter = new WriteOnlyDouble(0, "Horizontal Target Position (Meters)", Elevator.class.getSimpleName());
    
    public Elevator() {
        verticalLeftMotor = new CANSparkMax(16, MotorType.kBrushless);
        verticalRightElevator = new CANSparkMax(15, MotorType.kBrushless);//TODO: Where is it plugged in?
        horizontalMotor = new CANSparkMax(17, MotorType.kBrushless);

        verticalLeftMotor.setIdleMode(IdleMode.kCoast);
        verticalRightElevator.setIdleMode(IdleMode.kCoast);
        horizontalMotor.setIdleMode(IdleMode.kCoast);
        horizontalMotor.setInverted(true);

        verticalRightElevator.follow(verticalLeftMotor, true);
  
        verticalEncoder = verticalLeftMotor.getEncoder();  //TODO: Where is it plugged in?
        verticalEncoder.setPositionConversionFactor(Constants.VERTICAL_ROT_TO_INCHES);
        horizontalEncoder = horizontalMotor.getEncoder();
        horizontalEncoder.setPositionConversionFactor(Constants.HORIZONTAL_ROT_TO_INCHES);

        verticalController = new PIDController(0.01, 0, 0);
        verticalController.setTolerance(0.1); // meters//TODO:How much tolerence?
        horizontalController = new PIDController(0.05, 0, 0);
        horizontalController.setTolerance(0.10); // meters

        new ComplexWidgetBuilder(verticalController, "Vertical PID Controller", Elevator.class.getSimpleName())
            .withWidget(BuiltInWidgets.kPIDController)
            .withSize(2, 2);
        new ComplexWidgetBuilder(horizontalController, "Horizontal PID Controller", Elevator.class.getSimpleName())
            .withWidget(BuiltInWidgets.kPIDController)
            .withSize(2, 2);

        setPosition(Position.START);

        verticaAbsEncoder = new DutyCycleEncoder(9);
        isMovingManually = false;
    }

    private final WriteOnlyDouble horizontalSetPowerWriter = new WriteOnlyDouble(0.0, "horizontal set power", "Elevator");
    private final WriteOnlyDouble verticalSetPowerWriter = new WriteOnlyDouble(0.0, "vertical set power", "Elevator");
    
    // private final WriteOnlyDouble horizeEcoderReading = new WriteOnlyDouble(0.0, "horiz encoder reading", "Elevator");
    @Override
    public void periodic() {
        horizontalMotor.set(horizontalController.calculate(horizontalEncoder.getPosition()));
        horizontalSetPowerWriter.set(horizontalController.calculate(horizontalEncoder.getPosition()));

        // verticalLeftMotor.set(verticalController.calculate(verticalEncoder.getPosition()));
        // verticalSetPowerWriter.set(verticalController.calculate(verticalEncoder.getPosition()));

        horizontalEncoderPositionWriter.set(horizontalEncoder.getPosition());
        verticalEncoderPositionWriter.set(verticalEncoder.getPosition());
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

    public CommandBase setHorizontalPower(double power){
        return runOnce(() ->horizontalMotor.set(power));
    }
}  
