package frc.robot.utilities;

import java.lang.reflect.Type;
import java.util.Map;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;

public class SimpleWidgetBuilder<T> {
    private final SimpleWidget simpleWidget;
    private final T toAdd;

    private SimpleWidgetBuilder(T value, String title, String tab) {
        this.toAdd = value;
        this.simpleWidget = Shuffleboard.getTab(tab)
            .add(title, value);
    }

    public static <T> SimpleWidgetBuilder<?> create(T value, String title, String tab) {
        return new SimpleWidgetBuilder<T>(value, title, tab);
    }

    private SimpleWidgetBuilder(SimpleWidget simpleWidget, T toAdd) {
        this.simpleWidget = simpleWidget;
        this.toAdd = toAdd;
    }

    public SimpleWidgetBuilder<T> withPosition(int columnIndex, int rowIndex) {
        return new SimpleWidgetBuilder<T>(
            this.simpleWidget.withPosition(columnIndex, rowIndex), 
            this.toAdd
            );
    }

    public SimpleWidgetBuilder<T> withSize(int height, int width) {
        return new SimpleWidgetBuilder<T>(
            this.simpleWidget.withSize(height, width), 
            this.toAdd
            );
    }

    public SimpleWidgetBuilder<T> withProperties(Map<String, Object> properties) {
        return new SimpleWidgetBuilder<T>(
            this.simpleWidget.withProperties(properties), 
            this.toAdd
            );
    }

    public SimpleWidgetBuilder<T> withWidget(WidgetType widgetType) {
        return new SimpleWidgetBuilder<T>(
            this.simpleWidget.withWidget(widgetType), 
            this.toAdd
            );
    }

    public MutableDouble buildMutableDouble() {
        return new MutableDouble(simpleWidget.getEntry(), (double)toAdd);
    }
    
    public WriteOnlyDouble buildWriteOnlyDouble() {
        return new WriteOnlyDouble(simpleWidget.getEntry(), (double)toAdd);
    }

    public MutableString buildMutableString() {
        return new MutableString(simpleWidget.getEntry(), (String)toAdd);
    }
    
    public WriteOnlyString buildWriteOnlyString() {
        return new WriteOnlyString(simpleWidget.getEntry(), (String)toAdd);
    }

    public MutableBoolean buildMutableBoolean() {
        return new MutableBoolean(simpleWidget.getEntry(), (boolean)toAdd);
    }

    public WriteOnlyBoolean buildWriteOnlyBoolean() {
        return new WriteOnlyBoolean(simpleWidget.getEntry(), (boolean)toAdd);
    }
}