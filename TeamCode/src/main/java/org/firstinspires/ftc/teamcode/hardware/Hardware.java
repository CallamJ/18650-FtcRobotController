package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.teamcode.utilities.Pose;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Optional;

/** @noinspection MismatchedQueryAndUpdateOfCollection*/
public class Hardware {
    private final Map<DeviceKey<?>, Device> devices = new HashMap<>();
    private final List<Caching> caches = new ArrayList<>();
    private final HardwareMap hardwareMap;
    private final List<LynxModule> hubs;
    private LynxModule controlHub;
    private LynxModule expansionHub;

    public Hardware(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        this.hubs = hardwareMap.getAll(LynxModule.class);

        hubs.forEach(hub ->
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL)
        );

        hubs.forEach(hub -> {
            if(hub.isParent()){
                controlHub = hub;
            } else {
                expansionHub = hub;
            }
        });
    }

    public List<LynxModule> getHubs() {
        return hubs;
    }

    public SmartCamera getCamera(String name, Pose pose){
        Optional<SmartCamera> cameraOptional = getDevice(SmartCamera.class, name);
        if (cameraOptional.isPresent()){
            return cameraOptional.get();
        }

        SmartCamera camera = new SmartCamera(hardwareMap.get(CameraName.class, name), name, pose);
        return registerDevice(SmartCamera.class, camera);
    }

    public SmartLimelight3A getLimelight(String name) {
        Optional<SmartLimelight3A> limelightOptional = getDevice(SmartLimelight3A.class, name);
        if (limelightOptional.isPresent()){
            return limelightOptional.get();
        }

        SmartLimelight3A smartLimelight3A = new SmartLimelight3A(name, hardwareMap.get(Limelight3A.class, name));
        return registerCachedDevice(SmartLimelight3A.class, smartLimelight3A);
    }

    /**
     * Retrieves the hardware object for the given motor. If the motor has already been retrieved, this will return the same cached motor object.
     *
     * @param name the name of the motor.
     * @return the motor object associated with the passed name.
     */
    public SmartMotor getMotor(String name) {
        return getMotor(name, false);
    }

    /**
     * Retrieves the hardware object for the given motor. If the motor has already been retrieved, this will return the same cached motor object.
     *
     * @param name the name of the motor.
     * @return the motor object associated with the passed name.
     */
    public SmartMotor getMotor(String name, boolean hasExternalEncoder) {
        Optional<SmartMotor> motorOptional = getDevice(SmartMotor.class, name);
        if (motorOptional.isPresent()){
            return motorOptional.get();
        }

        SmartMotor smartMotor = new SmartMotor(hardwareMap.get(DcMotorEx.class, name), name, hasExternalEncoder);
        return registerCachedDevice(SmartMotor.class, smartMotor);
    }

    public SmartColorSensor getColorSensor(String name) {
        return getColorSensor(name, ColorMatchConfig.frontProfile());
    }

    public SmartColorSensor getColorSensor(String name, ColorMatchConfig.ColorMatchProfile colorProfile) {
        Optional<SmartColorSensor> colorSensorOptional = getDevice(SmartColorSensor.class, name);
        if (colorSensorOptional.isPresent()){
            return colorSensorOptional.get();
        }
        
        SmartColorSensor smartColorSensor =  new SmartColorSensor(
                hardwareMap.get(NormalizedColorSensor.class, name),
                name,
                colorProfile
        );
        return registerCachedDevice(SmartColorSensor.class, smartColorSensor);
    }
    
    public SmartServo getServo(String name){
        Optional<SmartServo> servoOptional = getDevice(SmartServo.class, name);
        if (servoOptional.isPresent()){
            return servoOptional.get();
        }

        SmartServo servo = new SmartServo(hardwareMap.get(Servo.class, name), name);
        return registerCachedDevice(SmartServo.class, servo);
    }

    public SmartLEDIndicator getLEDIndicator(String name) {
        Optional<SmartLEDIndicator> indicatorOptional = getDevice(SmartLEDIndicator.class, name);
        if (indicatorOptional.isPresent()) {
            return indicatorOptional.get();
        }

        SmartLEDIndicator indicator = new SmartLEDIndicator(getServo(name));
        return registerDevice(SmartLEDIndicator.class, indicator);
    }

    public SmartTouchSensor getTouchSensor(String name){
        Optional<SmartTouchSensor> touchSensorOptional = getDevice(SmartTouchSensor.class, name);
        if (touchSensorOptional.isPresent()){
            return touchSensorOptional.get();
        }

        SmartTouchSensor smartTouchSensor = new SmartTouchSensor(hardwareMap.get(TouchSensor.class, name), name);
        return registerCachedDevice(SmartTouchSensor.class, smartTouchSensor);
    }

    public SmartAnalogInput getAnalogInput(String name){
        Optional<SmartAnalogInput> inputOptional = getDevice(SmartAnalogInput.class, name);
        if (inputOptional.isPresent()){
	        return inputOptional.get();
        }

        SmartAnalogInput input = new SmartAnalogInput(hardwareMap.get(AnalogInput.class, name), name);
        return registerDevice(SmartAnalogInput.class, input);
    }

    public SmartPotentiometer getPotentiometer(String name, double maxAngle, double maxVoltage){
        Optional<SmartPotentiometer> potentiometerOptional = getDevice(SmartPotentiometer.class, name);
        if (potentiometerOptional.isPresent()){
            SmartPotentiometer potentiometer = potentiometerOptional.get();
            potentiometer.updateCache();
            return potentiometer;
        }

        SmartAnalogInput input = getAnalogInput(name);
        SmartPotentiometer potentiometer = new SmartPotentiometer(input, name, maxAngle, maxVoltage);
        registerCachedDevice(SmartPotentiometer.class, potentiometer);
        potentiometer.updateCache();
        return potentiometer;
    }

    public SmartPotentiometer getPotentiometer(String name, double maxAngle, double maxVoltage, double offset){
        Optional<SmartPotentiometer> potentiometerOptional = getDevice(SmartPotentiometer.class, name);
        if (potentiometerOptional.isPresent()){
            SmartPotentiometer potentiometer = potentiometerOptional.get();
            potentiometer.updateCache();
            return potentiometer;
        }

        SmartAnalogInput input = getAnalogInput(name);
        SmartPotentiometer potentiometer = new SmartPotentiometer(input, name, maxAngle, maxVoltage, offset);
        registerCachedDevice(SmartPotentiometer.class, potentiometer);
        potentiometer.updateCache();
        return potentiometer;
    }

    /**
     * @param type the hardware class which you would like to get. Only supports FTC-SDK classes.
     * @param name the name of the device to get.
     * @return the hardware object requested.
     */
    public <T> T getRaw(Class<? extends T> type, String name) {
        return hardwareMap.get(type, name);
    }

    public void invalidateCaches() {
        caches.forEach(Caching::invalidateCache);
        hubs.forEach(LynxModule::clearBulkCache);
    }

    public LynxModule getControlHub(){
        return controlHub;
    }

    public LynxModule getExpansionHub(){
        return expansionHub;
    }

    public void setCachingStrategy(Caching.Strategy strategy){
        caches.forEach(caching -> caching.setStrategy(strategy));
    }

    private <T extends Device> Optional<T> getDevice(Class<? extends T> type, String configName){
        return Optional.ofNullable(type.cast(devices.get(new DeviceKey<>(type, configName))));
    }

    private <T extends Device> T registerDevice(Class<T> type, T device) {
        devices.put(new DeviceKey<>(type, device.getConfigName()), device);
        return device;
    }

    private <T extends Device & Caching> T registerCachedDevice(Class<T> type, T device) {
        devices.put(new DeviceKey<>(type, device.getConfigName()), device);
        caches.add(device);
        return device;
    }

    private static final class DeviceKey<T extends Device> {
        private final Class<T> type;
        private final String configName;

        private DeviceKey(Class<T> type, String configName) {
            this.type = type;
            this.configName = configName;
        }

        @Override
        public boolean equals(Object obj) {
            if (this == obj) {
                return true;
            }
            if (!(obj instanceof DeviceKey)) {
                return false;
            }
            DeviceKey<?> other = (DeviceKey<?>) obj;
            return Objects.equals(type, other.type) && Objects.equals(configName, other.configName);
        }

        @Override
        public int hashCode() {
            return Objects.hash(type, configName);
        }
    }
}
