package org.firstinspires.ftc.teamcode.drive.advanced;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@TeleOp
public class LinkedMotorTuner extends LinearOpMode {

    //ftc-JjOY
    public static PIDCoefficients MOTOR_VELO_PID = new PIDCoefficients(0.0061, 0, 0.000035);

//    public static PIDCoefficients MOTOR_VELO_PID = new PIDCoefficients(0.004, 0, 0.0002);


//    public static PIDCoefficients MOTOR_VELO_PID = new  PIDCoefficients(0.001, 0, 0.00001);

    //public static double kV = 0.00034;//0.000427// 0.00015;

    public static double kV = 1 / TuningController.rpmToTicksPerSecond(TuningController.MOTOR_MAX_RPM);
    public static double kA = 0.00015; //0.000366
    public static double kStatic = 0;



    //ftc-Chineze
//    public static PIDCoefficients MOTOR_VELO_PID = new PIDCoefficients(0, 0, 0);
//
//    public static double kV = 1 / TuningController.rpmToTicksPerSecond(TuningController.MOTOR_MAX_RPM);
//    public static double kA = 0;
//    public static double kStatic = 0;

    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    private final ElapsedTime veloTimer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        // Change my id
        DcMotorEx myMotor1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        DcMotorEx myMotor2 = hardwareMap.get(DcMotorEx.class, "shooter2");

        // Reverse as appropriate
        myMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
        myMotor2.setDirection(DcMotorSimple.Direction.REVERSE);

        myMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        myMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        VelocityPIDFController veloController = new VelocityPIDFController(MOTOR_VELO_PID, kV, kA, kStatic);
        TuningController tuningController = new TuningController();

        double lastTargetVelo = 0.0;
        double lastKv = kV;
        double lastKa = kA;
        double lastKstatic = kStatic;

        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        telemetry.addLine("Ready!");
        telemetry.update();
        telemetry.clearAll();

        waitForStart();

        if (isStopRequested()) return;

        tuningController.start();
        veloTimer.reset();

        while (!isStopRequested() && opModeIsActive()) {
            double targetVelo = tuningController.update();

            veloController.setTargetVelocity(targetVelo);
            veloController.setTargetAcceleration((targetVelo - lastTargetVelo) / veloTimer.seconds());
            veloTimer.reset();

            lastTargetVelo = targetVelo;

            telemetry.addData("targetVelocity", targetVelo);

            double motorPos = myMotor2.getCurrentPosition();
            double motorVelo = myMotor2.getVelocity();

            double power = veloController.update(motorPos, motorVelo);
            myMotor1.setPower(power);
            myMotor2.setPower(power);

            if(lastKv != kV || lastKa != kA || lastKstatic != kStatic) {
                lastKv = kV;
                lastKa = kA;
                lastKstatic = kStatic;

                veloController = new VelocityPIDFController(MOTOR_VELO_PID, kV, kA, kStatic);
            }

            telemetry.addData("velocity", motorVelo);
            telemetry.addData("error", targetVelo - motorVelo);

            telemetry.addData("upperBound", TuningController.rpmToTicksPerSecond(TuningController.TESTING_MAX_SPEED * 1.15));
            telemetry.addData("lowerBound", 0);
            telemetry.update();
        }
    }
}