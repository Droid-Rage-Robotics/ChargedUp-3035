package frc.robot.subsystems;

import frc.robot.utilities.MutableDouble;
import frc.robot.utilities.SimpleWidgetBuilder;
import frc.robot.utilities.WriteOnlyString;

public abstract class Position {
    public enum Positions {//16-17 is MAXXXXXX for vert ; 11 is for horiz
        START(0,0,0),

        INTAKELOWCONE(0,0, 51),
        INTAKELOWCUBE(0,0, 49.9),
       
        LOWCONE(0, 0, 31.4),
        LOWCUBE(0,0, 43.1),
        
        MIDCONE(13.2,11,LOWCONE.pivotAngle.get()),
        MIDCUBE(13.4,10.4, 53),

        AUTOMIDCONE(15.2, 11.5, MIDCONE.pivotAngle.get()),
        // AUTOMIDCUBE(13.4,10.4, MIDCONE.pivotAngle.get()),

        HIGHCONE(14.4,11, LOWCONE.pivotAngle.get()),
        HIGHCUBE(17,11, 31),

        INTAKEHIGH1CONE(14.9,0, 37),
        INTAKEHIGH1CUBE(14.9,0, 36.9),

        INTAKEHIGH2CONE(13.5,0, 20),
        INTAKEHIGH2CUBE(13.5,0,20),

        HOLD(0,0,-22),
        
        ;

        private MutableDouble verticalInches;
        private MutableDouble horizontalInches;
        private MutableDouble pivotAngle;

        private Positions(double verticalInches, double horizontalInches, double pivotAngle) {
            this.verticalInches = SimpleWidgetBuilder.create(verticalInches, Positions.class.getSimpleName()+"/"+name()+"/Vertical (Inches)", Elevator.class.getSimpleName())
                .withSize(1, 3)
                .buildMutableDouble();
            this.horizontalInches = SimpleWidgetBuilder.create(horizontalInches, Positions.class.getSimpleName()+"/"+name()+"/Horizontal (Inches)", Elevator.class.getSimpleName())
                .withSize(1, 3)
                .buildMutableDouble();
            this.pivotAngle = SimpleWidgetBuilder.create(pivotAngle, Positions.class.getSimpleName()+"/"+name()+"/Pivot Angle", Elevator.class.getSimpleName())
                .withSize(1, 3)
                .buildMutableDouble();
        }

        public Positions get(){
            return position;
        }
    }
    
    public static volatile Positions position = Positions.START;
    private static final WriteOnlyString PositionWriter = new WriteOnlyString(position.name(), "Position", "Misc");
    
    public static Positions get() {
        return Position.position;
    }

    // public static boolean isCone() {
    //     return element == Positions.CONE;
    // }

    public static void set(Positions position) {
        PositionWriter.set(position.name());
        Position.position = position; // discord
    }

    public static double getPivotDegrees(Positions position) {
        return position.pivotAngle.get();
        // Position.position = position; // maybe have the writers of the positions in here as well
    }

    public static double getVerticalHeight(Positions position) {
        return position.verticalInches.get();
        // Position.position = position; // maybe have the writers of the positions in here as well
    }
    public static double getHorizonotalDistance(Positions position) {
        return position.horizontalInches.get();
        // Position.position = position; // maybe have the writers of the positions in here as well
    }
}
