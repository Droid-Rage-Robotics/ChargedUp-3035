package frc.robot.subsystem.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.SuppliedCommand;
import frc.robot.subsystem.Intake;
import frc.robot.subsystem.TrackedElement;
import frc.robot.subsystem.arm.elevator.HorizontalElevator;
import frc.robot.subsystem.arm.elevator.VerticalElevator;
import frc.robot.subsystem.arm.pivot.Pivot;
import frc.robot.utility.ShuffleboardValue;

public class Arm {
    public enum Position {
        START(Value.START, Value.START),
        INTAKE_LOW(Value.INTAKE_LOW_CONE, Value.INTAKE_LOW_CUBE),
        INTAKE_LOW_DROPPED(Value.INTAKE_LOW_CONE_DROPPED, Value.INTAKE_LOW_CUBE),
        LOW(Value.LOW_CONE, Value.LOW_CUBE),
        MID(Value.MID_CONE, Value.MID_CUBE),
        AUTO_HIGH(Value.AUTO_HIGH_CONE, Value.AUTO_HIGH_CUBE),
        AUTO_MID(Value.AUTO_MID_CONE, Value.AUTO_MID_CUBE),
        AUTO_LOW(Value.LOW_CONE, Value.LOW_CUBE),
        HIGH(Value.HIGH_CONE, Value.HIGH_CUBE),
        INTAKE_HIGH_DOUBLE_SUBSTATION(Value.INTAKE_HIGH_DOUBLE_SUBSTATION_CONE, Value.INTAKE_HIGH_DOUBLE_SUBSTATION_CUBE),
        INTAKE_HIGH_SINGLE_SUBSTATION(Value.INTAKE_HIGH_SINGLE_SUBSTATION_CONE, Value.INTAKE_HIGH_SINGLE_SUBSTATION_CUBE),
        HOLD(Value.HOLD, Value.HOLD),
        AUTO_HOLD(Value.AUTO_HOLD, Value.AUTO_HOLD),
        AUTO_INTAKE_LOW(Value.AUTO_INTAKE_LOW_CONE, Value.AUTO_INTAKE_LOW_CUBE)
        ;
        private final Value coneValue;
        private final Value cubeValue;

        private Position(Value coneValue, Value cubeValue) {
            this.coneValue = coneValue;
            this.cubeValue = cubeValue;
        }

        public Value getCurrentValue() {
            return switch (TrackedElement.get()) {
                case CONE -> coneValue;
                case CUBE -> cubeValue;
            };
        }

        public double getVertical() {
            return getCurrentValue().getVertical();
        }

        public double getHorizontal() {
            return getCurrentValue().getHorizontal();
        }

        public double getPivotDegrees() {
            return getCurrentValue().getPivotDegrees();
        }
    }
    public enum Value {//16-17 is MAXXXXXX for vert ; 11 is for horiz
        START(0,0,0),

        INTAKE_LOW_CONE(0,0, 213),//215 or 219 (shoot COnes)
        INTAKE_LOW_CUBE(0,0, 209),
        INTAKE_LOW_CONE_DROPPED(0.,2, 230),

        AUTO_INTAKE_LOW_CONE(0,13, INTAKE_LOW_CONE.getPivotDegrees()),//215 or 219 (shoot COnes)
        AUTO_INTAKE_LOW_CUBE(0,13, INTAKE_LOW_CUBE.getPivotDegrees()),
       
        LOW_CONE(0, 0, 200),
        LOW_CUBE(0,0, 145),

        MID_CONE(13.,11.3,172), 
        // MID_CUBE(0,0, 125), - shoot mid place
        MID_CUBE(13.4,10.4, 193),

        AUTO_MID_CONE(14.6, 11.5, 175),
        AUTO_MID_CUBE(13,10, 170),

        HIGH_CONE(16,12.5, 145.5),
        HIGH_CUBE(15.1,12.65, 153),

        AUTO_HIGH_CONE(HIGH_CONE),
        AUTO_HIGH_CUBE(HIGH_CUBE),

        INTAKE_HIGH_DOUBLE_SUBSTATION_CONE(13.5,0, 181),
        INTAKE_HIGH_DOUBLE_SUBSTATION_CUBE(13.2,0, 175),

        INTAKE_HIGH_SINGLE_SUBSTATION_CONE(12,0, 110),
        INTAKE_HIGH_SINGLE_SUBSTATION_CUBE(12,0,113),

        HOLD(0,0, 60),
        AUTO_HOLD(HOLD.getVertical(),HOLD.getHorizontal(), HOLD.getPivotDegrees()),
        
        ;

        private final ShuffleboardValue<Double> verticalInches;
        private final ShuffleboardValue<Double> horizontalInches;
        private final ShuffleboardValue<Double> pivotAngle;

        private Value(double verticalInches, double horizontalInches, double pivotAngleDegrees) {
            this.verticalInches = ShuffleboardValue.create(verticalInches, Position.class.getSimpleName()+"/"+name()+"/Vertical (Inches)", "Misc")
                .withSize(1, 3)
                .build();
            this.horizontalInches = ShuffleboardValue.create(horizontalInches, Position.class.getSimpleName()+"/"+name()+"/Horizontal (Inches)", "Misc")
                .withSize(1, 3)
                .build();
            this.pivotAngle = ShuffleboardValue.create(pivotAngleDegrees, Position.class.getSimpleName()+"/"+name()+"/Pivot Angle", "Misc")
                .withSize(1, 3)
                .build();
        }

        // This constructor lets you get it from another value
        private Value(Value copyValue) {
            this.verticalInches = ShuffleboardValue.create(copyValue.getVertical(), Position.class.getSimpleName()+"/"+name()+"/Vertical (Inches)", "Misc")
                .withSize(1, 3)
                .build();
            this.horizontalInches = ShuffleboardValue.create(copyValue.getHorizontal(), Position.class.getSimpleName()+"/"+name()+"/Horizontal (Inches)", "Misc")
                .withSize(1, 3)
                .build();
            this.pivotAngle = ShuffleboardValue.create(copyValue.getPivotDegrees(), Position.class.getSimpleName()+"/"+name()+"/Pivot Angle", "Misc")
                .withSize(1, 3)
                .build();
        }

        public double getVertical() {
            return verticalInches.get();
        }

        public double getHorizontal() {
            return horizontalInches.get();
        }

        public double getPivotDegrees() {
            return pivotAngle.get();
        }

        public void setVertical(double inches) {
            verticalInches.set(inches);
        }

        public void setHorizontal(double inches) {
            horizontalInches.set(inches);
        }

        public void setPivotDegrees(double degrees) {
            pivotAngle.set(degrees);
        }
    }

    private final VerticalElevator verticalElevator;
    private final HorizontalElevator horizontalElevator;
    private final Pivot pivot;
    private final Intake intake;
    private Position position = Position.START;
    private final ShuffleboardValue<String> positionWriter = ShuffleboardValue.create(position.name(), "Current Arm Position", "Misc")
        .withSize(1, 3)
        .build();

    public Arm(VerticalElevator verticalElevator, HorizontalElevator horizontalElevator, Pivot pivot, Intake intake) {
        this.verticalElevator = verticalElevator;
        this.horizontalElevator = horizontalElevator;
        this.pivot = pivot;
        this.intake = intake;
    }

    private void logPosition(Position targetPosition) {
        position = targetPosition;
        positionWriter.write(position.name());
        // positionWriter.write(targetPosition.name());
    }

    public Position getPosition() {
        return position;
    }

    public CommandBase setPositionCommand(Position targetPosition) {
        return SuppliedCommand.create(() -> Commands.sequence(
            Commands.runOnce(() -> logPosition(targetPosition)),
            switch (targetPosition) {
                case AUTO_HIGH -> //Commands.sequence( seperate
                    new SequentialCommandGroup(
                        verticalElevator.runOnce(() -> verticalElevator.setTargetPosition(targetPosition.getVertical())),
                        Commands.waitSeconds(1),
                        horizontalElevator.runOnce(() -> horizontalElevator.setTargetPosition(targetPosition.getHorizontal())),
                        Commands.waitSeconds(0.8),
                        pivot.runOnce(() -> pivot.setTargetPosition(Math.toRadians(targetPosition.getPivotDegrees())))
                    );
                case  AUTO_MID ->
                    new SequentialCommandGroup(
                        verticalElevator.runOnce(() -> verticalElevator.setTargetPosition(targetPosition.getVertical())),
                        Commands.waitSeconds(0.5),
                        horizontalElevator.runOnce(() -> horizontalElevator.setTargetPosition(targetPosition.getHorizontal())),
                        // Commands.waitSeconds(0.3),
                        pivot.runOnce(() -> pivot.setTargetPosition(Math.toRadians(targetPosition.getPivotDegrees())))
                    );
                case AUTO_HOLD, HOLD -> //Commands.sequence(
                    new ParallelCommandGroup(//TODO:ONly mve pivot fgor the cube not cone
                        verticalElevator.runOnce(() -> verticalElevator.setTargetPosition(targetPosition.getVertical())),
                        horizontalElevator.runOnce(() -> horizontalElevator.setTargetPosition(targetPosition.getHorizontal())),
                        pivot.runOnce(() -> pivot.setTargetPosition(Math.toRadians(targetPosition.getPivotDegrees())))
                    );

                case INTAKE_HIGH_DOUBLE_SUBSTATION, INTAKE_HIGH_SINGLE_SUBSTATION, INTAKE_LOW, START, AUTO_INTAKE_LOW -> //Commands.sequence(
                    new ParallelCommandGroup(
                        verticalElevator.runOnce(() -> verticalElevator.setTargetPosition(targetPosition.getVertical())),
                        horizontalElevator.runOnce(() -> horizontalElevator.setTargetPosition(targetPosition.getHorizontal())),
                        pivot.runOnce(() -> pivot.setTargetPosition(Math.toRadians(targetPosition.getPivotDegrees())))
                    );
                
                default -> 
                    new ParallelCommandGroup(
                        verticalElevator.runOnce(() -> verticalElevator.setTargetPosition(targetPosition.getVertical())),
                        horizontalElevator.runOnce(() -> horizontalElevator.setTargetPosition(targetPosition.getHorizontal())),
                        pivot.runOnce(() -> pivot.setTargetPosition(Math.toRadians(targetPosition.getPivotDegrees())))
                        // intake.runOnce(()->intake.close(false))//TODO: FIXXXXXX
                    );
            }
        ));
    }

    public CommandBase lowerElevatorCommand() {
        return verticalElevator.runOnce(() -> verticalElevator.setTargetPosition(verticalElevator.getTargetPosition() - 3));
    }
    public CommandBase lowerPivotCommand(double lowerNum) {
        return pivot.runOnce(() -> pivot.setTargetPosition(pivot.getTargetPosition() - lowerNum));
    }
    public CommandBase pushChargeDownCommand() {
        return pivot.runOnce(() -> pivot.setTargetPosition(Value.INTAKE_LOW_CUBE.getPivotDegrees()));
    }
}
