package org.firstinspires.ftc.teamcode.subsystems.gantry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Gantry{

	public DcMotorEx gantryMotor;

	PIDController gantryPID;
	public static double kP = 0;
	public static double kI = 0;
	public static double kD = 0;

	public static double DOCK_POSTION = 0;
	public static double DRIVER_POSTION_MIN = 0;
	public static double DRIVER_POSTION_MAX = 0;
	public static double DRIVER_POSITON_RANGE = DRIVER_POSTION_MAX - DRIVER_POSTION_MIN;


	Telemetry telemetry;
	TelemetryPacket packet = new TelemetryPacket();
	FtcDashboard dashboard = FtcDashboard.getInstance();

	public Gantry(HardwareMap map, Telemetry telemetry) {
		this.telemetry = telemetry;

		gantryMotor = map.get(DcMotorEx.class, "gantryMotor");
		gantryMotor.setDirection(DcMotorEx.Direction.REVERSE);
		gantryMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		gantryMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

		gantryPID = new PIDController(kP, kI, kD);

		telemetry.addData("Gantry", "Initialized");
		telemetry.update();

		packet.put("Gantry", "Initialized");
		dashboard.sendTelemetryPacket(packet);
	}

	public void setGantryPower(double power) {
		gantryMotor.setPower(power);
	}

	public void stop() {
		gantryMotor.setPower(0);
	}

	public double getPosition() {
		return gantryMotor.getCurrentPosition();
	}

	public void setPositon(double position) {
		gantryPID.setSetPoint(position);
	}

	public void updateGantryPID() {
		gantryPID.setP(kP);
		gantryPID.setI(kI);
		gantryPID.setD(kD);
	}

	public void update() {
		updateGantryPID();
		double output = Range.clip(
				gantryPID.calculate(getPosition()),
				-1,
				1
		);

		gantryMotor.setPower(output);
	}
}
