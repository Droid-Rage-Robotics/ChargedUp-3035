package frc.robot.utilities;

import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;
import java.io.Serializable;
import java.util.Objects;

import javax.xml.validation.Validator;
import javax.xml.validation.ValidatorHandler;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;

public abstract class ShuffleboardValue<T> {
    protected final GenericEntry entry;
    protected final T defaultValue;

    private static GenericEntry createGenericEntry(Object defaultValue, String title, String tab) {
        return Shuffleboard.getTab(tab)
            .add(title, defaultValue)
            .getEntry();
    }

    private ShuffleboardValue(GenericEntry entry, T defaultValue) {
        this.entry = entry;
        this.defaultValue = defaultValue;
    }

    public abstract T get();
    public abstract void set(T value);
    public void write(T value) {
        assert value != null;
        set(value);
    }

    static ShuffleboardValue<Double> create(GenericEntry entry, double defaultValue) {
        return new ShuffleboardValue<Double>(entry, defaultValue) {
            @Override
            public synchronized Double get() {
                return entry.getDouble(defaultValue);
            }

            @Override
            public synchronized void set(Double value) {
                entry.setDouble(value);
            }
        };
    }

    static ShuffleboardValue<Double> build(double defaultValue, String title, String tab) {
        return ShuffleboardValue.create(
            ShuffleboardValue.createGenericEntry(defaultValue, title, tab),
            defaultValue);
    }

    static ShuffleboardValue<Long> create(GenericEntry entry, long defaultValue) {
        return new ShuffleboardValue<Long>(entry, defaultValue) {
            @Override
            public synchronized Long get() {
                return entry.getInteger(defaultValue);
            }

            @Override
            public synchronized void set(Long value) {
                Objects.requireNonNull(value);
                entry.setInteger(value);
            }
        };
    }

    static ShuffleboardValue<Long> build(long defaultValue, String title, String tab) {
        return ShuffleboardValue.create(
            ShuffleboardValue.createGenericEntry(defaultValue, title, tab),
            defaultValue);
    }

    static ShuffleboardValue<Boolean> create(GenericEntry entry, boolean defaultValue) {
        return new ShuffleboardValue<Boolean>(entry, defaultValue) {
            @Override
            public synchronized Boolean get() {
                return entry.getBoolean(defaultValue);
            }

            @Override
            public synchronized void set(Boolean value) {
                Objects.requireNonNull(value);
                entry.setBoolean(value);
            }
        };
    }

    static ShuffleboardValue<Boolean> build(boolean defaultValue, String title, String tab) {
        return ShuffleboardValue.create(
            ShuffleboardValue.createGenericEntry(defaultValue, title, tab),
            defaultValue);
    }

    static ShuffleboardValue<String> create(GenericEntry entry, String defaultValue) {
        return new ShuffleboardValue<String>(entry, defaultValue) {
            @Override
            public synchronized String get() {
                return entry.getString(defaultValue);
            }

            @Override
            public synchronized void set(String value) {
                Objects.requireNonNull(value);
                entry.setString(value);
            }
        };
    }

    static ShuffleboardValue<String> build(String defaultValue, String title, String tab) {
        return ShuffleboardValue.create(
            ShuffleboardValue.createGenericEntry(defaultValue, title, tab),
            defaultValue);
    }

    static<T> ShuffleboardValue<Serializable> create(GenericEntry entry, Serializable defaultValue){
        return new ShuffleboardValue<Serializable>(entry, defaultValue) {
            @Override
            public synchronized Serializable get() {
                return convertBytesToObject(
                    entry.getRaw(convertObjectToBytes(defaultValue))
                );
            }

            @Override
            public synchronized void set(Serializable value) {
                Objects.requireNonNull(value);
                entry.setRaw(convertObjectToBytes(defaultValue));
            }
        };
    }

    private static byte[] convertObjectToBytes(Serializable obj) {
        ByteArrayOutputStream boas = new ByteArrayOutputStream();
        try (ObjectOutputStream ois = new ObjectOutputStream(boas)) {
            ois.writeObject(obj);
            return boas.toByteArray();
        } catch (IOException ioe) {
            ioe.printStackTrace();
        }
        throw new RuntimeException();
    }

    private static Serializable convertBytesToObject(byte[] bytes) {
        InputStream is = new ByteArrayInputStream(bytes);
        try (ObjectInputStream ois = new ObjectInputStream(is)) {
            return (Serializable) ois.readObject();
        } catch (IOException | ClassNotFoundException ioe) {
            ioe.printStackTrace();
        }
        throw new RuntimeException();
    }

    public static ShuffleboardValueBuilder<Double> create(double defaultValue, String title, String tab) {
        return new ShuffleboardValueBuilder<Double>(defaultValue, title, tab) {
            @Override
            public ShuffleboardValue<Double> build() {
                return ShuffleboardValue.create(simpleWidget.getEntry(), defaultValue);
            }

            @Override
            protected ShuffleboardValueBuilder<Double> add(SimpleWidget simpleWidget) {
                this.simpleWidget = simpleWidget;
                return this;
            }
        };
    }

    public static ShuffleboardValueBuilder<Long> create(long defaultValue, String title, String tab) {
        return new ShuffleboardValueBuilder<Long>(defaultValue, title, tab) {
            @Override
            public ShuffleboardValue<Long> build() {
                return ShuffleboardValue.create(simpleWidget.getEntry(), defaultValue);
            }

            @Override
            protected ShuffleboardValueBuilder<Long> add(SimpleWidget simpleWidget) {
                this.simpleWidget = simpleWidget;
                return this;
            }
        };
    }

    public static ShuffleboardValueBuilder<Boolean> create(boolean defaultValue, String title, String tab) {
        return new ShuffleboardValueBuilder<Boolean>(defaultValue, title, tab) {
            @Override
            public ShuffleboardValue<Boolean> build() {
                return ShuffleboardValue.create(simpleWidget.getEntry(), defaultValue);
            }

            @Override
            protected ShuffleboardValueBuilder<Boolean> add(SimpleWidget simpleWidget) {
                this.simpleWidget = simpleWidget;
                return this;
            }
        };
    }

    public static ShuffleboardValueBuilder<String> create(String defaultValue, String title, String tab) {
        return new ShuffleboardValueBuilder<String>(defaultValue, title, tab) {
            @Override
            public ShuffleboardValue<String> build() {
                return ShuffleboardValue.create(simpleWidget.getEntry(), defaultValue);
            }

            @Override
            protected ShuffleboardValueBuilder<String> add(SimpleWidget simpleWidget) {
                this.simpleWidget = simpleWidget;
                return this;
            }
        };
    }
}
