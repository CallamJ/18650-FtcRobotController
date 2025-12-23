package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.teamcode.utilities.Pose;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.stream.Collectors;

/** @noinspection MismatchedQueryAndUpdateOfCollection*/
public class Hardware {
    private static final List<SmartCamera> cameras = new ArrayList<>();
    private static final List<SmartMotor> motors = new ArrayList<>();
    private static final List<SmartColorSensor> colorSensors = new ArrayList<>();
    private static final List<SmartTouchSensor> touchSensors = new ArrayList<>();
    private static final List<SmartServo> servos = new ArrayList<>();
    private static final List<SmartPotentiometer> potentiometers = new ArrayList<>();
    private static final List<SmartAnalogInput> analogInputs = new ArrayList<>();
    private static final List<Device> devices = new ArrayList<>();
    private static final List<Caching> caches = new ArrayList<>();

    private static HardwareMap hardwareMap;
    private static List<LynxModule> hubs;

    public static void init(HardwareMap hardwareMap) {
        Hardware.hardwareMap = hardwareMap;
        Hardware.hubs = hardwareMap.getAll(LynxModule.class);

        devices.clear();
        caches.clear();
        servos.clear();
        cameras.clear();
        motors.clear();
        colorSensors.clear();
        touchSensors.clear();
        potentiometers.clear();
        analogInputs.clear();

        hubs.forEach(hub ->
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO)
        );
    }

    public static List<LynxModule> getHubs() {
        assertInitialized();
        return hubs;
    }

    public static SmartCamera getCamera(String name, Pose pose){
        assertInitialized();

        Optional<SmartCamera> cameraOptional = getDevice(SmartCamera.class, name);
        if (cameraOptional.isPresent()){
            return cameraOptional.get();
        }

        SmartCamera camera = new SmartCamera(hardwareMap.get(CameraName.class, name), name, pose);
        cameras.add(camera);
        devices.add(camera);
        return camera;
    }

    /**
     * Retrieves the hardware object for the given motor. If the motor has already been retrieved, this will return the same cached motor object.
     *
     * @param name the name of the motor.
     * @return the motor object associated with the passed name.
     */
    public static SmartMotor getMotor(String name) {
        return getMotor(name, false);
    }

    /**
     * Retrieves the hardware object for the given motor. If the motor has already been retrieved, this will return the same cached motor object.
     *
     * @param name the name of the motor.
     * @return the motor object associated with the passed name.
     */
    public static SmartMotor getMotor(String name, boolean hasExternalEncoder) {
        assertInitialized();

        Optional<SmartMotor> motorOptional = getDevice(SmartMotor.class, name);
        if (motorOptional.isPresent()){
            return motorOptional.get();
        }

        SmartMotor smartMotor = new SmartMotor(hardwareMap.get(DcMotorEx.class, name), name, hasExternalEncoder);
        motors.add(smartMotor);
        devices.add(smartMotor);
        caches.add(smartMotor);
        return smartMotor;
    }

    public static SmartColorSensor getColorSensor(String name) {
        assertInitialized();

        Optional<SmartColorSensor> colorSensorOptional = getDevice(SmartColorSensor.class, name);
        if (colorSensorOptional.isPresent()){
            return colorSensorOptional.get();
        }
        
        SmartColorSensor smartColorSensor =  new SmartColorSensor(hardwareMap.get(NormalizedColorSensor.class, name), name);
        colorSensors.add(smartColorSensor);
        devices.add(smartColorSensor);
        caches.add(smartColorSensor);
        return smartColorSensor;
    }
    
    public static SmartServo getServo(String name){
        assertInitialized();

        Optional<SmartServo> servoOptional = getDevice(SmartServo.class, name);
        if (servoOptional.isPresent()){
            return servoOptional.get();
        }

        SmartServo servo = new SmartServo(hardwareMap.get(Servo.class, name), name);
        servos.add(servo);
        devices.add(servo);
        caches.add(servo);
        return servo;
    }

    public static SmartTouchSensor getTouchSensor(String name){
        assertInitialized();

        Optional<SmartTouchSensor> touchSensorOptional = getDevice(SmartTouchSensor.class, name);
        if (touchSensorOptional.isPresent()){
            return touchSensorOptional.get();
        }

        SmartTouchSensor smartTouchSensor = new SmartTouchSensor(hardwareMap.get(TouchSensor.class, name), name);
        touchSensors.add(smartTouchSensor);
        devices.add(smartTouchSensor);
        caches.add(smartTouchSensor);
        return smartTouchSensor;
    }

    public static SmartAnalogInput getAnalogInput(String name){
        assertInitialized();

        Optional<SmartAnalogInput> inputOptional = getDevice(SmartAnalogInput.class, name);
        if (inputOptional.isPresent()){
	        return inputOptional.get();
        }

        SmartAnalogInput input = new SmartAnalogInput(hardwareMap.get(AnalogInput.class, name), name);
        analogInputs.add(input);
        return input;
    }

    public static SmartPotentiometer getPotentiometer(String name, double maxAngle, double maxVoltage){
        assertInitialized();

        Optional<SmartPotentiometer> potentiometerOptional = getDevice(SmartPotentiometer.class, name);
        if (potentiometerOptional.isPresent()){
            SmartPotentiometer potentiometer = potentiometerOptional.get();
            potentiometer.updateCache();
            return potentiometer;
        }

        SmartAnalogInput input = getAnalogInput(name);
        SmartPotentiometer potentiometer = new SmartPotentiometer(input, name, maxAngle, maxVoltage);
        potentiometers.add(potentiometer);
        devices.add(potentiometer);
        caches.add(potentiometer);
        potentiometer.updateCache();
        return potentiometer;
    }

    public static SmartPotentiometer getPotentiometer(String name, double maxAngle, double maxVoltage, double offset){
        assertInitialized();

        Optional<SmartPotentiometer> potentiometerOptional = getDevice(SmartPotentiometer.class, name);
        if (potentiometerOptional.isPresent()){
            SmartPotentiometer potentiometer = potentiometerOptional.get();
            potentiometer.updateCache();
            return potentiometer;
        }

        SmartAnalogInput input = getAnalogInput(name);
        SmartPotentiometer potentiometer = new SmartPotentiometer(input, name, maxAngle, maxVoltage, offset);
        potentiometers.add(potentiometer);
        devices.add(potentiometer);
        caches.add(potentiometer);
        potentiometer.updateCache();
        return potentiometer;
    }

    /**
     * @param type the hardware class which you would like to get. Only supports FTC-SDK classes.
     * @param name the name of the device to get.
     * @return the hardware object requested.
     */
    public static <T> T getOther(Class<? extends T> type, String name) {
        assertInitialized();
        return hardwareMap.get(type, name);
    }

    public static void invalidateCaches() {
        assertInitialized();
        caches.forEach(Caching::invalidateCache);
    }

    public static void setCachingStrategy(Caching.Strategy strategy){
        assertInitialized();
        caches.forEach(caching -> caching.setStrategy(strategy));
    }

    private static void assertInitialized() {
        if (hardwareMap == null)
            throw new HardwareMapNotInitializedException();
    }

    private static Optional<Device> getFirstDevice(String configName){
        return devices.stream().filter(device -> device.configName.equals(configName)).findFirst();
    }

    private static List<Device> getDevices(String configName){
        return devices.stream().filter(device -> device.configName.equals(configName)).collect(Collectors.toList());
    }

    private static <T extends Device> Optional<T> getDevice(Class<? extends T> type, String configName){
	    //noinspection unchecked
	    return (Optional<T>) devices.stream()
                .filter(device -> device.configName.equals(configName))
                .filter(device -> device.getClass().equals(type))
                .findFirst();
    }
}
