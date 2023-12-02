package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class MainTeleOp extends LinearOpMode {

    // slides
    public double slideKg = 0;
    public double pivotHeight = 500;

    // pivots
    public double flatPivot = 0;
    public double anglePivot = 0.5;

    // claw
    public double clawOpen = 1;
    public double clawClosed = 0;


    private DcMotorEx driveBL;
    private DcMotorEx driveBR;
    private DcMotorEx driveFL;
    private DcMotorEx driveFR;

    private DcMotorEx slidesMotor;

    private Servo claw;
    private Servo clawPivot;

    private boolean clawBtnPressed = false;
    private boolean isClawOpen = false;

    @Override
    public void runOpMode() throws InterruptedException {

        initDrivetrain();
        initSlides();
        initClaw();
        waitForStart();

        while(opModeIsActive()) {
            updateSlides();
            updateDrivetrain();

            if(gamepad2.a && !clawBtnPressed) {
                isClawOpen = !isClawOpen;
                setClaw(isClawOpen);
            }
            clawBtnPressed = gamepad2.a;
        }


    }

    private void initClaw() {
        claw = hardwareMap.get(Servo.class, "claw");
        clawPivot = hardwareMap.get(Servo.class, "clawPivot");

        setClawPivot(true);
        setClaw(false);
    }

    private void setClaw(boolean open) {
        if(open) {
            claw.setPosition(clawOpen);
        } else {
            claw.setPosition(clawClosed);
        }
    }

    private void setClawPivot(boolean flat) {
        if(flat) {
            clawPivot.setPosition(flatPivot);
        } else {
            clawPivot.setPosition(anglePivot);
        }
    }

    private void initSlides() {
        slidesMotor = hardwareMap.get(DcMotorEx.class, "slides");

        slidesMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidesMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slidesMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        slidesMotor.setDirection(DcMotorEx.Direction.REVERSE);

    }

    private void updateSlides() {

        double inputPower = -gamepad2.left_stick_y;
        slidesMotor.setPower(inputPower + slideKg);

        setClawPivot(slidesMotor.getCurrentPosition() < pivotHeight);

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
