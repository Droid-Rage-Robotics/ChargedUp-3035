package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.SuppliedCommand;
import frc.robot.subsystems.Pivot.Pivot;
import frc.robot.utilities.ShuffleboardValue;

public class Arm {
    public enum Position {
        START(Value.START, Value.START),
        INTAKE_LOW(Value.INTAKE_LOW_CONE, Value.INTAKE_LOW_CUBE),
        LOW(Value.LOW_CONE, Value.LOW_CUBE),
        MID(Value.MID_CONE, Value.MID_CUBE),
        AUTO_MID(Value.AUTO_MID_CONE, Value.AUTO_MID_CUBE),
        HIGH(Value.HIGH_CONE, Value.HIGH_CUBE),
        INTAKE_HIGH_DOUBLE_SUBSTATION(Value.INTAKE_HIGH_DOUBLE_SUBSTATION_CONE, Value.INTAKE_HIGH_DOUBLE_SUBSTATION_CUBE),
        INTAKE_HIGH_SINGLE_SUBSTATION(Value.INTAKE_HIGH_SINGLE_SUBSTATION_CONE, Value.INTAKE_HIGH_SINGLE_SUBSTATION_CUBE),
        HOLD(Value.HOLD, Value.HOLD),
        ;
        private final Value coneValue;
        private final Value cubeValue;

        private Position(Value coneValue, Value cubeValue) {
            this.coneValue = coneValue;
            this.cubeValue = cubeValue;
        }
        private Value getCurrentValue() {
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
    private enum Value {//16-17 is MAXXXXXX for vert ; 11 is for horiz
        START(0,0,0),

        INTAKE_LOW_CONE(0,0, 51),
        INTAKE_LOW_CUBE(0,0, 49.9),
       
        LOW_CONE(0, 0, 31.4),
        LOW_CUBE(0,0, 43.1),
        
        MID_CONE(13.2,11,31.4), // LOWCONE.pivotAngle.get() 
        MID_CUBE(13.4,10.4, 53),

        AUTO_MID_CONE(15.2, 11.5, 31.4),// MIDCONE.pivotAngle.get() // up first then out
        AUTO_MID_CUBE(15.2, 11.5, 31.4),// MIDCONE.pivotAngle.get() // up first then out
        // AUTOMIDCUBE(13.4,10.4, MIDCONE.pivotAngle.get()),

        HIGH_CONE(15.5,11, 31.4),// LOWCONE.pivotAngle.get()
        HIGH_CUBE(15,11, 31),

        INTAKE_HIGH_DOUBLE_SUBSTATION_CONE(14.9,0, 37),
        INTAKE_HIGH_DOUBLE_SUBSTATION_CUBE(14.9,0, 36.9),

        INTAKE_HIGH_SINGLE_SUBSTATION_CONE(13.5,0, 20),
        INTAKE_HIGH_SINGLE_SUBSTATION_CUBE(13.5,0,20),

        HOLD(0,0, -22),
        
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

        public double getVertical() {
            return verticalInches.get();
        }

        public double getHorizontal() {
            return horizontalInches.get();
        }

        public double getPivotDegrees() {
            return pivotAngle.get();
        }
    }

    private final Elevator elevator;
    private final Pivot pivot;
    private Position position = Position.START;
    private final ShuffleboardValue<String> positionWriter = ShuffleboardValue.create(position.name(), "Current Arm Position", "Misc")
        .withSize(1, 3)
        .build();

    public Arm(Elevator elevator, Pivot pivot) {
        this.elevator = elevator;
        this.pivot = pivot;
    }

    private void logPosition(Position targetPosition) {
        position = targetPosition;
        positionWriter.write(targetPosition.name());
    }

    public Position getPosition() {
        return position;
    }

    public CommandBase setPositionCommand(Position targetPosition) {
        return new SuppliedCommand(() -> Commands.sequence(
            Commands.runOnce(() -> logPosition(targetPosition)),
            switch (targetPosition) {
                case AUTO_MID -> Commands.sequence(
                    elevator.runOnce(() -> elevator.setPositions(targetPosition.getHorizontal(), targetPosition.getVertical())),
                    Commands.waitSeconds(1.4),
                    pivot.runOnce(() -> pivot.setTargetPosition(Math.toRadians(targetPosition.getPivotDegrees())))
                ); 
                
                default -> Commands.sequence(
                    elevator.runOnce(() -> elevator.setPositions(targetPosition.getHorizontal(), targetPosition.getVertical())),
                    pivot.runOnce(() -> pivot.setTargetPosition(Math.toRadians(targetPosition.getPivotDegrees())))
                );
            }
        ));
    }

    public CommandBase lowerElevatorCommand() {
        return elevator.runOnce(() -> elevator.setPositions(position.getHorizontal(), position.getVertical() - 7));
    }
}
