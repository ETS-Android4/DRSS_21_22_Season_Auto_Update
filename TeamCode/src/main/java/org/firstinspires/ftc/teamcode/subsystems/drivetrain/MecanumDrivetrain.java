package org.firstinspires.ftc.teamcode.subsystems.drivetrain;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MecanumDrivetrain{

	public DcMotorEx FL;
	public DcMotorEx FR;
	public DcMotorEx BL;
	public DcMotorEx BR;

	public MecanumDrivetrain(HardwareMap map) {
		FL = map.get(DcMotorEx.class, "FL");
		FR = map.get(DcMotorEx.class, "FR");
		BL = map.get(DcMotorEx.class, "BL");
		BR = map.get(DcMotorEx.class, "BR");

		FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
	}

	public void setPowers(double FLPower, double FRPower, double BLPower, double BRPower) {
		FL.setPower(FLPower);
		FR.setPower(FRPower);
		BL.setPower(BLPower);
		BR.setPower(BRPower);
	}

	public void stop() {
		FL.setPower(0);
		FR.setPower(0);
		BL.setPower(0);
		BR.setPower(0);
	}
}
