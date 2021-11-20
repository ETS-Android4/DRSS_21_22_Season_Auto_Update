package org.firstinspires.ftc.teamcode.subsystems.controls;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Controls{

	public TriggerReader speedTrigger;

	public Controls(GamepadEx gamepad1ex, GamepadEx gamepad2ex) {

		speedTrigger = new TriggerReader(
				gamepad1ex, GamepadKeys.Trigger.RIGHT_TRIGGER
		);

	}

	public void readValues() {
		speedTrigger.readValue();
	}
}
