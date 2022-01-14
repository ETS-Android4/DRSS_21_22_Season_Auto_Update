package org.firstinspires.ftc.teamcode.subsystems.gantry;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.PIDEx;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficientsEx;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Gantry{

	public DcMotorEx gantryMotor;

	PIDEx gantryPID;
	public static double kP = -0.0035;
	public static double kI = -0.000;
	public static double kD = 0;
	double integralSumMax = 1 / kI;
	double stabilityThreshold = 0.0;
	double lowPassGain = 0.0;
	PIDCoefficientsEx gantryPIDCoefficients;

	double PINION_DIAMETER = 3.8315;
	double PINION_CIRCUMFERENCE = PINION_DIAMETER * 3.14159;
	double COUNTS_PER_ROTATION = 288;
	double COUNTS_PER_INCH = PINION_CIRCUMFERENCE * COUNTS_PER_ROTATION;

	public double DOCK_POSTION = -40;
	public double DRIVER_POSTION_MIN = -110;
	public double DRIVER_POSTION_MAX = -245;
	public double DRIVER_POSITON_RANGE = DRIVER_POSTION_MAX - DRIVER_POSTION_MIN;


	Telemetry telemetry;
	TelemetryPacket packet = new TelemetryPacket();
	FtcDashboard dashboard = FtcDashboard.getInstance();

	public Gantry(HardwareMap map, Telemetry telemetry) {
		this.telemetry = telemetry;

		gantryMotor = map.get(DcMotorEx.class, "gantryMotor");
		gantryMotor.setDirection(DcMotorEx.Direction.REVERSE);
		gantryMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		gantryMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

		gantryPIDCoefficients = new PIDCoefficientsEx(kP, kI, kD, integralSumMax, stabilityThreshold, lowPassGain);
		gantryPID = new PIDEx(gantryPIDCoefficients);

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

	public void updateGantryPID() {
		gantryPIDCoefficients.Kp = kP;
		gantryPIDCoefficients.Ki = kI;
		gantryPIDCoefficients.Kd = kD;
		gantryPIDCoefficients.maximumIntegralSum = 1 / kI;
	}

	public void update(double setPoint) {
		updateGantryPID();
		double output = Range.clip(
				gantryPID.calculate(setPoint, gantryMotor.getCurrentPosition()),
				-1,
				1
		);

		gantryMotor.setPower(output);
	}
}
