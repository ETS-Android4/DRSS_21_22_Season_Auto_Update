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

	public TriggerReader intakeTrigger;
	public TriggerReader outtakeTrigger;

	public Controls(GamepadEx gamepad1ex, GamepadEx gamepad2ex) {

		/*Speed Controls*/
		speedTrigger = new TriggerReader(
				gamepad1ex, GamepadKeys.Trigger.RIGHT_TRIGGER
		);

		quarterSpeedButton = new ButtonReader(
				gamepad1ex, GamepadKeys.Button.X
		);

		fullSpeedButton = new ButtonReader(
				gamepad1ex, GamepadKeys.Button.Y
		);

		/*Intake Controls*/
		intakeTrigger = new TriggerReader(
				gamepad2ex, GamepadKeys.Trigger.LEFT_TRIGGER
		);

		outtakeTrigger = new TriggerReader(
				gamepad2ex, GamepadKeys.Trigger.RIGHT_TRIGGER
		);

	}

	public void readValues() {
		speedTrigger.readValue();
		quarterSpeedButton.readValue();
		fullSpeedButton.readValue();

		intakeTrigger.readValue();
		outtakeTrigger.readValue();
	}
}
