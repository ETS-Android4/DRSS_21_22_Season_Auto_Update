package org.firstinspires.ftc.teamcode.subsystems.lift;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.PIDEx;
import com.ThermalEquilibrium.homeostasis.Filters.Estimators.Estimator;
import com.ThermalEquilibrium.homeostasis.Filters.Estimators.KalmanEstimator;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficientsEx;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.controller.PController;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.function.DoubleSupplier;

@Config
public class Lift{

	DcMotorEx liftMotor;
	DistanceSensor liftHeightSensor;

	public double setPoint = 0;

	PIDEx liftPID;
	PIDCoefficientsEx liftPIDCoefficients;
	public static double kP = 0.3;
	public static double kI = 0.0;
	public static double kD = 0.0;
	double integralSumMax = 1 / kI;
	double stabilityThreshold = 0.0;
	double lowPassGain = 0.0;

	public static double Q = 0.3;
	public static double R = 3;
	public static int N = 3;
	DoubleSupplier heightSensor = new DoubleSupplier(){
		@Override
		public double getAsDouble(){
			return getHeight();
		}
	};
	Estimator heightFilter = new KalmanEstimator(heightSensor, Q, R, N);

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

		liftPIDCoefficients = new PIDCoefficientsEx(kP, kI, kD, integralSumMax, stabilityThreshold, lowPassGain);
		liftPID = new PIDEx(liftPIDCoefficients);

		telemetry.addData("Lift", "Initialized");
		telemetry.update();

		packet.put("Lift", "Initialized");
		dashboard.sendTelemetryPacket(packet);
	}

	public void setLiftPower(double power) {
		liftMotor.setPower(power);
	}

	public void stop() {
		liftMotor.setPower(0);
	}

	public double getHeight() {
		return (liftHeightSensor.getDistance(DistanceUnit.INCH) - Z_OFFSET);
	}

	public double getFilteredHeight() {
		return heightFilter.update();
	}

	public void setHeight(double inches) {
		setPoint = inches;
	}

	public void updateLiftPID() {
		liftPIDCoefficients.Kp = kP;
		liftPIDCoefficients.Ki = kI;
		liftPIDCoefficients.Kd = kD;
		liftPIDCoefficients.maximumIntegralSum = integralSumMax;
		liftPIDCoefficients.stabilityThreshold = stabilityThreshold;
		liftPIDCoefficients.lowPassGain = lowPassGain;
	}

	public void update() {
		updateLiftPID();
		double output = Range.clip(
				liftPID.calculate(setPoint, getHeight()),
				-1.0,
				1.0
		);

		liftMotor.setPower(output);
	}
}
