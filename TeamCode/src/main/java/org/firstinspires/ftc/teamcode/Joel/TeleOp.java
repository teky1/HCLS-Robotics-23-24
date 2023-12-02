package org.firstinspires.ftc.teamcode.Joel;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

public class TeleOp extends LinearOpMode {

    // tune until slides don't fall
    public double slideKg = 0;


    private DcMotorEx driveBL;
    private DcMotorEx driveBR;
    private DcMotorEx driveFL;
    private DcMotorEx driveFR;

    private DcMotorEx slidesMotor;

    @Override
    public void runOpMode() throws InterruptedException {

        initDrivetrain();
        initOuttake();
        waitForStart();

        while(opModeIsActive()) {
            updateOuttake();
            updateDrivetrain();
        }


    }

    private void initOuttake() {

        slidesMotor = hardwareMap.get(DcMotorEx.class, "slides");
        slidesMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slidesMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        slidesMotor.setDirection(DcMotorEx.Direction.REVERSE);

    }

    private void updateOuttake() {

        double inputPower = -gamepad2.left_stick_y;
        slidesMotor.setPower(inputPower + slideKg);

    }

    private void initDrivetrain() {

        driveBL = hardwareMap.get(DcMotorEx.class, "backLeft");
        driveBR = hardwareMap.get(DcMotorEx.class, "backRight");
        driveFL = hardwareMap.get(DcMotorEx.class, "frontLeft");
        driveFR = hardwareMap.get(DcMotorEx.class, "frontRight");

        driveBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        driveBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // SWAP THESE IF NEEDED
        driveBL.setDirection(DcMotorEx.Direction.REVERSE);
//        driveBR.setDirection(DcMotorEx.Direction.REVERSE);
        driveFL.setDirection(DcMotorEx.Direction.REVERSE);
//        driveFR.setDirection(DcMotorEx.Direction.REVERSE);

    }

    private void updateDrivetrain() {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x * 1.1;
        double rx = gamepad1.right_stick_x;
        double d = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        driveBL.setPower((y - x + rx) / d);
        driveBR.setPower((y + x - rx) / d);
        driveFL.setPower((y + x + rx) / d);
        driveFR.setPower((y - x - rx) / d);
    }
}
