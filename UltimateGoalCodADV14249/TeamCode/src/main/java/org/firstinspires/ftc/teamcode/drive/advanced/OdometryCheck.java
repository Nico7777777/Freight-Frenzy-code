package org.firstinspires.ftc.teamcode.drive.advanced;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class OdometryCheck extends LinearOpMode {

    DcMotor lf,rf,lr,rr,intake,shooter1,shooter2,wobble;

    @Override
    public void runOpMode() throws InterruptedException {

        wobble= hardwareMap.get(DcMotor.class, "wobble");
        shooter1 = hardwareMap.get(DcMotor.class,"shooter1");
        shooter2 = hardwareMap.get(DcMotor.class,"shooter2");
        intake = hardwareMap.get(DcMotor.class,"intake");
        lf = hardwareMap.get(DcMotor.class,"leftFront");
        lr = hardwareMap.get(DcMotor.class,"leftRear");
        rf = hardwareMap.get(DcMotor.class,"rightFront");
        rr = hardwareMap.get(DcMotor.class,"rightRear");


        waitForStart();

        while (opModeIsActive()){
            telemetry.addData("lf:  ", lf.getCurrentPosition());
            telemetry.addData("rf:  ", rf.getCurrentPosition());
            telemetry.addData("lr:  ", lr.getCurrentPosition());
            telemetry.addData("rr:  ", rr.getCurrentPosition());

            telemetry.addData("intake:  ", intake.getCurrentPosition());
            telemetry.addData("wobble:  ", wobble.getCurrentPosition());
            telemetry.addData("shooter1:  ", shooter1.getCurrentPosition());
            telemetry.addData("shooter2:  ", shooter2.getCurrentPosition());

            telemetry.update();
        }

    }
}