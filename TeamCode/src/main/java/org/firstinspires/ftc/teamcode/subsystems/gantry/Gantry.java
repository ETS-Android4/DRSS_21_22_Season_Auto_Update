package org.firstinspires.ftc.teamcode.subsystems.gantry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Gantry{

	DcMotorEx gantryMotor;

	PIDFController gantryPIDF;
	double kP = 1;
	double kI = 1;
	double kD = 1;
	double kF = 1;

	Telemetry telemetry;
	TelemetryPacket packet = new TelemetryPacket();
	FtcDashboard dashboard = FtcDashboard.getInstance();

	public Gantry(HardwareMap map, Telemetry telemetry) {
		this.telemetry = telemetry;

		gantryMotor = map.get(DcMotorEx.class, "gantryMotor");
		gantryMotor.setDirection(DcMotorEx.Direction.REVERSE);
		//gantryMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		//gantryMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

		//gantryPIDF = new PIDFController(kP, kI, kD, kF);

		telemetry.addData("Gantry", "Initialized");
		telemetry.update();

		packet.put("Gantry", "Initialized");
		dashboard.sendTelemetryPacket(packet);
	}

	public void setGantryPower(double power) {
		gantryMotor.setPower(power);
		return;
	}

	public void setGantryVelocity(int velocity) {
		gantryMotor.setVelocity(velocity);
		return;
	}

	public void stop() {
		gantryMotor.setPower(0);

		return;
	}
}
