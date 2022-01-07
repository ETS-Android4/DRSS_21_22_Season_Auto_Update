package org.firstinspires.ftc.teamcode.subsystems.intake;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Intake{

	public DcMotorEx intakeMotor;

	public ColorSensor freightSensor;

	public static double COLOR_THRESHOLD = .5;

	Telemetry telemetry;
	TelemetryPacket packet = new TelemetryPacket();
	FtcDashboard dashboard = FtcDashboard.getInstance();

	public Intake(HardwareMap map, Telemetry telemetry) {
		this.telemetry = telemetry;

		intakeMotor = map.get(DcMotorEx.class, "intakeMotor");
		intakeMotor.setDirection(DcMotorEx.Direction.FORWARD);

		freightSensor = map.get(ColorSensor.class, "freightSensor");

		telemetry.addData("Intake", "Initialized");
		telemetry.update();

		packet.put("Intake", "Initialized");
		dashboard.sendTelemetryPacket(packet);
	}

	public void runIntake(double speed) {
		intakeMotor.setPower(speed);
	}

	public void stop() {
		intakeMotor.setPower(0);
	}

	public boolean isLoaded() {
		if (freightSensor.red() > COLOR_THRESHOLD) {
			return true;
		}
		else {
			return false;
		}
	}
}
