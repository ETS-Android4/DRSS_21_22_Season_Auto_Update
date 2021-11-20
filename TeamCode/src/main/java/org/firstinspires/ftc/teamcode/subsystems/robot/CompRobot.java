package org.firstinspires.ftc.teamcode.subsystems.robot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.gantry.Gantry;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.pusher.Pusher;

public class CompRobot{
	public Gantry gantry;
	public Intake intake;
	public Pusher pusher;

	Telemetry telemetry;
	TelemetryPacket packet = new TelemetryPacket();
	FtcDashboard dashboard = FtcDashboard.getInstance();

	public CompRobot(HardwareMap map, Telemetry telemetry) {
		this.telemetry = telemetry;

		gantry= new Gantry(map, telemetry);
		intake = new Intake(map, telemetry);
		pusher = new Pusher(map, telemetry);

	}
}
