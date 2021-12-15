package org.firstinspires.ftc.teamcode.subsystems.lift;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Lift{

	DcMotorEx liftMotor;

	PIDFController liftPIDF;
	double kP = 1;
	double kI = 1;
	double kD = 1;
	double kF = 1;

	Telemetry telemetry;
	TelemetryPacket packet = new TelemetryPacket();
	FtcDashboard dashboard = FtcDashboard.getInstance();

	public Lift(HardwareMap map, Telemetry telemetry) {
		this.telemetry = telemetry;

		liftMotor = map.get(DcMotorEx.class, "liftMotor");
		liftMotor.setDirection(DcMotorEx.Direction.FORWARD);
		liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		//liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		//liftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

		//liftPIDF = new PIDFController(kP, kI, kD, kF);

		telemetry.addData("Lift", "Initialized");
		telemetry.update();

		packet.put("Lift", "Initialized");
		dashboard.sendTelemetryPacket(packet);
	}

	public void setLiftPower(double power) {
		liftMotor.setPower(power);

		return;
	}

	public void stop() {
		liftMotor.setPower(0);

		return;
	}
}
