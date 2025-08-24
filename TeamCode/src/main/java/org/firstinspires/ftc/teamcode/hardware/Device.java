package org.firstinspires.ftc.teamcode.hardware;

public abstract class Device {
	protected final String configName;
	protected Device(String configName){
		this.configName = configName;
	}

	public String getConfigName(){
		return configName;
	}
}
