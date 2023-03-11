package frc.robot.utilities;

import java.util.Map;

import edu.wpi.first.cscore.VideoSource;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;

public class ComplexWidgetBuilder {
    private final ComplexWidget complexWidget;

    private ComplexWidgetBuilder(Sendable toAdd, String title, String tab) {
        this.complexWidget = Shuffleboard.getTab(tab)
            .add(title, toAdd);
    }

    public static ComplexWidgetBuilder create(Sendable toAdd, String title, String tab) {
        return new ComplexWidgetBuilder(toAdd, title, tab);
    }

    private ComplexWidgetBuilder(VideoSource videoSource, String title, String tab) {
        this.complexWidget = Shuffleboard.getTab(tab)
            .add(title, videoSource);
    }

    public static ComplexWidgetBuilder create(VideoSource videoSource, String title, String tab) {
        return new ComplexWidgetBuilder(videoSource, title, tab);
    }

    private ComplexWidgetBuilder(ComplexWidget complexWidget) {
        this.complexWidget = complexWidget;
    }

    public ComplexWidgetBuilder withPosition(int columnIndex, int rowIndex) {
        return new ComplexWidgetBuilder(
            this.complexWidget.withPosition(columnIndex, rowIndex)
            );
    }

    public ComplexWidgetBuilder withSize(int height, int width) {
        return new ComplexWidgetBuilder(
            this.complexWidget.withSize(width, height)
            );
    }

    public ComplexWidgetBuilder withProperties(Map<String, Object> properties) {
        return new ComplexWidgetBuilder(
            this.complexWidget.withProperties(properties)
            );
    }

    public ComplexWidgetBuilder withWidget(WidgetType widgetType) {
        return new ComplexWidgetBuilder(
            this.complexWidget.withWidget(widgetType)
            );
    }
    public void get() {
        
    }
}