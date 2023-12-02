package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous
public class AutoLeft extends LinearOpMode {

    private DcMotorEx driveBL;
    private DcMotorEx driveBR;
    private DcMotorEx driveFL;
    private DcMotorEx driveFR;

    public void runOpMode() {
        initDrivetrain();
        waitForStart();

        int distance = -(int)(48*Math.sqrt(2)*384.5/11.871);

        driveBL.setTargetPosition(-distance);
        driveBR.setTargetPosition(distance);
        driveFL.setTargetPosition(distance);
        driveFR.setTargetPosition(-distance);

    }

    private void initDrivetrain() {

        driveBL = hardwareMap.get(DcMotorEx.class, "backLeft");
        driveBR = hardwareMap.get(DcMotorEx.class, "backRight");
        driveFL = hardwareMap.get(DcMotorEx.class, "frontLeft");
        driveFR = hardwareMap.get(DcMotorEx.class, "frontRight");

        driveBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        driveBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        driveBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        driveFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        driveFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

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

}
