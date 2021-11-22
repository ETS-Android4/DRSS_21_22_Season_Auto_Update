package org.firstinspires.ftc.teamcode.subsystems.controls;

import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Controls{

	public TriggerReader speedTrigger;
	public ButtonReader quarterSpeedButton;
	public ButtonReader fullSpeedButton;

	public Controls(GamepadEx gamepad1ex, GamepadEx gamepad2ex) {

		speedTrigger = new TriggerReader(
				gamepad1ex, GamepadKeys.Trigger.RIGHT_TRIGGER
		);

		quarterSpeedButton = new ButtonReader(
				gamepad1ex, GamepadKeys.Button.X
		);

		fullSpeedButton = new ButtonReader(
				gamepad1ex, GamepadKeys.Button.Y
		);

	}

	public void readValues() {
		speedTrigger.readValue();
		quarterSpeedButton.readValue();
		fullSpeedButton.readValue();
	}
}
