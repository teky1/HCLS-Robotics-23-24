package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class ScrimTeleOp extends LinearOpMode {

    public DcMotorEx frontLeft, frontRight, backLeft, backRight, slidesMotor;
    public Servo pivotServo, leftClaw, rightClaw;
    double x, y, r;
    boolean clawPressed = false, clawOpen = false;

    public enum States {
        PIVOTOUT,
        OPEN,
        CLOSE,
        PIVOTIN
    }
    @Override
    public void runOpMode() throws InterruptedException {
        initDrivetrain();
        initServo();
        waitForStart();

        ElapsedTime time = new ElapsedTime();
        States outtakeState = States.PIVOTIN;
        double outtakeTime = time.milliseconds();

        while(opModeIsActive()){
            x = gamepad1.left_stick_x;
            y = gamepad1.left_stick_y;
            r = gamepad1.right_stick_x;

            frontLeft.setPower(y + x + r);
            frontRight.setPower(y - x - r);
            backLeft.setPower(y - x + r);
            backRight.setPower(y + x - r);

            double inputPower = 0.4*-gamepad2.left_stick_y;
            slidesMotor.setPower(inputPower);

            if(outtakeState == States.PIVOTIN){
                if(gamepad2.right_bumper && !clawPressed){
                    clawPressed = true;
                    clawOpen = !clawOpen;
                    controlClaw(clawOpen);
                }
                clawPressed = gamepad2.right_bumper;
            }

            switch(outtakeState){
                case PIVOTIN:
                    //close();
                    pivotServo.setPosition(0.03);
                    if(gamepad2.a){
                        outtakeState = States.PIVOTOUT;
                        outtakeTime = time.milliseconds();
                    }
                    break;
                case PIVOTOUT:
                    pivotServo.setPosition(0.25);
                    close();
                    if(time.milliseconds() > outtakeTime + 500)
                        outtakeState = States.OPEN;
                    break;
                case OPEN:
                    pivotServo.setPosition(0.25);
                    open();
                    if(time.milliseconds() > outtakeTime + 1000)
                        outtakeState = States.CLOSE;
                    break;
                case CLOSE:
                    pivotServo.setPosition(0.25);
                    close();
                    if(time.milliseconds() > outtakeTime + 1500)
                        outtakeState = States.PIVOTIN;
                    break;
            }

        }
    }

    public void initDrivetrain(){
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        slidesMotor = hardwareMap.get(DcMotorEx.class, "slidesMotor");

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slidesMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slidesMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void initServo(){
        pivotServo = hardwareMap.get(Servo.class, "pivotServo");
        leftClaw = hardwareMap.get(Servo.class, "leftClaw");
        rightClaw = hardwareMap.get(Servo.class, "rightClaw");
    }

    public void open(){
        leftClaw.setPosition(0.55);
        rightClaw.setPosition(0.35);
    }

    public void close(){
        leftClaw.setPosition(0.35);
        rightClaw.setPosition(0.55);
    }

    public void controlClaw(boolean open){
        if(open){
            open();
        } else {
            close();
        }
    }
}
