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

	public PIDEx gantryPID;
	PIDCoefficientsEx gantryPIDCoefficients;
	public double kP = -0.05;
	public double kI = 0;
	public double kD = 0;
	double intSumMax = 0;
	double stabilityThreshold = 0;
	double lowPassGain = 0;

	public boolean atSetPoint = false;

	double PINION_DIAMETER = 3.8315;
	double PINION_CIRCUMFERENCE = PINION_DIAMETER * 3.14159;
	double COUNTS_PER_ROTATION = 288;
	double COUNTS_PER_INCH = PINION_CIRCUMFERENCE * COUNTS_PER_ROTATION;

	double HOME_POSITION = 40;
	public double DOCK_POSTION = 0;
	public double DRIVER_POSTION_MIN = -150;
	public double DRIVER_POSTION_MAX = -285;
	public double DRIVER_POSITON_RANGE = DRIVER_POSTION_MAX - DRIVER_POSTION_MIN;

	//double lastResetPosition = 0;


	Telemetry telemetry;

	public Gantry(HardwareMap map, Telemetry telemetry, Boolean home) {
		this.telemetry = telemetry;

		gantryMotor = map.get(DcMotorEx.class, "gantryMotor");
		gantryMotor.setDirection(DcMotorEx.Direction.REVERSE);
		gantryMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		reset();

		gantryPIDCoefficients = new PIDCoefficientsEx(kP, kI, kD, intSumMax, stabilityThreshold, lowPassGain);
		gantryPID = new PIDEx(gantryPIDCoefficients);

		if (home) {
			while (gantryMotor.getCurrentPosition() < (HOME_POSITION-1)) {
				update(HOME_POSITION);

				telemetry.addData("position", gantryMotor.getCurrentPosition());
				telemetry.update();
			}
			telemetry.addData("Resetting", gantryMotor.getCurrentPosition());
			telemetry.update();
			reset();
			telemetry.addData("Reset", gantryMotor.getCurrentPosition());
			telemetry.update();
		}
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
	}

	public void update(double setPoint) {
		updateGantryPID();

		//double offsetSetPoint = setPoint - lastResetPosition;

		double output = Range.clip(
				gantryPID.calculate(setPoint, gantryMotor.getCurrentPosition()),
				-1,
				1
		);

		gantryMotor.setPower(output);
	}

	public void reset() {
		gantryMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		gantryMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
	}
}
