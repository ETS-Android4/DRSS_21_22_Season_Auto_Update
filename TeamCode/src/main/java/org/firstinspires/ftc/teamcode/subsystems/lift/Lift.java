package org.firstinspires.ftc.teamcode.subsystems.lift;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.controller.PController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
public class Lift{

	DcMotorEx liftMotor;
	DistanceSensor liftHeightSensor;

	PController liftPIDF;
	public static double kP = 0.3;

	public static double Z_OFFSET = 0.8;

	Telemetry telemetry;
	TelemetryPacket packet = new TelemetryPacket();
	FtcDashboard dashboard = FtcDashboard.getInstance();

	public Lift(HardwareMap map, Telemetry telemetry) {
		this.telemetry = telemetry;

		liftMotor = map.get(DcMotorEx.class, "liftMotor");
		liftMotor.setDirection(DcMotorEx.Direction.FORWARD);
		liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

		liftHeightSensor = map.get(DistanceSensor.class, "liftHeightSensor");

		liftPIDF = new PController(kP);

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

	public double getHeight() {
		return (liftHeightSensor.getDistance(DistanceUnit.INCH)-Z_OFFSET);
	}

	public void setHeight(double height) {
		liftPIDF.setSetPoint(height + Z_OFFSET);
	}

	public void update() {
		double output = Range.clip(liftPIDF.calculate(getHeight()), -1, 1);

		liftMotor.setPower(output);
		updateLiftP();
	}

	public void updateLiftP() {
		liftPIDF.setP(kP);
	}
}
