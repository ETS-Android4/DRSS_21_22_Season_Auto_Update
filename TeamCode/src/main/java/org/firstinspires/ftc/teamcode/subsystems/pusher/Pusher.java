package org.firstinspires.ftc.teamcode.subsystems.pusher;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Pusher{

	ServoEx pushServo;

	int DEFAULT_MIN_ANGLE = 0;
	int DEFAULT_MAX_ANGLE = 180;

	Telemetry telemetry;
	TelemetryPacket packet = new TelemetryPacket();
	FtcDashboard dashboard = FtcDashboard.getInstance();

	public Pusher(HardwareMap map, Telemetry telemetry) {
		this.telemetry = telemetry;

		pushServo = new SimpleServo(map, "pushServo",
			DEFAULT_MIN_ANGLE, DEFAULT_MAX_ANGLE,
			AngleUnit.DEGREES);

		pusherSetPosition(0);

		telemetry.addData("Pusher", "Initialized");
		telemetry.update();

		packet.put("Pusher", "Initialized");
		dashboard.sendTelemetryPacket(packet);
	}

	public void pusherSetPosition(float angle) {
		pushServo.turnToAngle(angle);
	}

	public void pusherSetPosition(double position) {
		pushServo.setPosition(position);
	}

	public void pusherMove(float angle) {
		pushServo.rotateByAngle(angle);
	}

	public void pusherMove(double distance) {
		pushServo.rotateBy(distance);
	}
}
