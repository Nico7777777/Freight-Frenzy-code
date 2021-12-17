package org.firstinspires.ftc.teamcode.drive.advanced;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Libs.GAMEPAD;

public class MechanismRed {

    private DcMotor shooter1;
    private DcMotorEx shooter2;
    private DcMotor intake;
    private DcMotor wobble;

    private Servo servo;
    private Servo wobble_servo;
    //    private Servo sabie;
    private Servo antena;

    private GAMEPAD gamepad1;
    private GAMEPAD gamepad2;
    private Telemetry telemetry;



    // Copy your PID Coefficients here
    public static PIDCoefficients MOTOR_VELO_PID = new PIDCoefficients(0.004, 0, 0.0002);

    // Copy your feedforward gains here
    public static double kV = 0.000427;
    public static double kA = 0.000366;
    public static double kStatic = 0;

    // Timer for calculating desired acceleration
    // Necessary for kA to have an affect
    private final ElapsedTime veloTimer = new ElapsedTime();
    private double lastTargetVelo = 0.0;

    // Our velocity controller
    private final VelocityPIDFController veloController = new VelocityPIDFController(MOTOR_VELO_PID, kV, kA, kStatic);



    public MechanismRed(HardwareMap hardwareMap, GAMEPAD gamepad1 , GAMEPAD gamepad2, Telemetry telemetry){
        //public Mechanism(HardwareMap hardwareMap, GAMEPAD gamepad1, Telemetry telemetry){
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.telemetry = telemetry;

        initMechanism(hardwareMap);
    }

    private void initMechanism(HardwareMap hardwareMap){
        shooter1 = hardwareMap.get(DcMotor.class, "shooter1");
        shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        intake = hardwareMap.get(DcMotor.class, "intake");
        wobble = hardwareMap.get(DcMotor.class, "wobble");

        servo = hardwareMap.get(Servo.class , "servo");
        wobble_servo = hardwareMap.get(Servo.class , "wobble_servo");
//        sabie = hardwareMap.get(Servo.class , "sabie");
        antena = hardwareMap.get(Servo.class , "antena");


        servo.setPosition(1);


        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wobble.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



//        sabie.setDirection(Servo.Direction.REVERSE);
        servo.setDirection(Servo.Direction.REVERSE);
        shooter1.setDirection(DcMotor.Direction.REVERSE);
        shooter2.setDirection(DcMotor.Direction.REVERSE);

        //wobble_servo.setDirection(Servo.Direction.REVERSE);

        intake.setDirection(DcMotorSimple.Direction.REVERSE);


        shooter1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

    }

    public void mechanism(){
        intake();
        trage();
        servo();
        wobble();
        wobble_servo();

        antena();

        telemetry.addData("velocity: ",shooter2.getVelocity());


    }
//    private void updateWobble(){
//        lastwobblex = this.wobblex;
//        this.wobblex = wobblex;
//    }

    private void intake(){
        if(gamepad1.right_trigger > 0.3 || gamepad2.right_trigger > 0.3)
            intake.setPower(-1);
        else if(gamepad1.left_trigger > 0.3 || gamepad2.left_trigger > 0.3)
            intake.setPower(1);
        else
            intake.setPower(0);
    }

    private void trage(){
        if(gamepad1.y.toggle){
            pidTrage(2169);
        }
        else if(gamepad2.x.toggle){
            pidTrage(2050);
        }
        else{
            shooter2.setPower(0);
            shooter1.setPower(0);
        }
    }

    private void pidTrage(double target){
        veloTimer.reset();

        double targetVelo = target;

        // Call necessary controller methods
        veloController.setTargetVelocity(targetVelo);
        veloController.setTargetAcceleration((targetVelo - lastTargetVelo) / veloTimer.seconds());
        veloTimer.reset();

        lastTargetVelo = targetVelo;

        // Get the velocity from the motor with the encoder
        double motorPos = shooter2.getCurrentPosition();
        double motorVelo = shooter2.getVelocity();

        // Update the controller and set the power for each motor
        double power = veloController.update(motorPos, motorVelo);
        shooter1.setPower(power);
        shooter2.setPower(power);

    }

    //
//    private void wobble(){
//        if(gamepad2.x.toggle) {
//            wobble.setTargetPosition(1750);
//            wobble.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                wobble.setPower(1);
//        }else if(gamepad2.a.toggle) {
//            wobble.setTargetPosition(0);
//            wobble.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            wobble.setPower(-1);
//        }else
//            wobble.setPower(0);
//    }
    private void wobble(){
        if(gamepad1.dpad_up.value || gamepad2.dpad_up.value){
            wobble.setPower(-1);
        }
        else if(gamepad1.dpad_down.value || gamepad2.dpad_down.value){
            wobble.setPower(1);
        }
        else{
            wobble.setPower(0);
        }
    }

    //    FTC-JjOY
  /*  private void servo(){
        if(gamepad1.right_bumper.value)
            servo.setPosition(0.5);
        else{
            servo.setPosition(0.1);
        }
    }*/

    //    FTC-Chineese
    private void servo(){
        if(gamepad1.right_bumper.value) {
            servo.setPosition(0.5);
        }
        else{
            servo.setPosition(0);
        }
    }

    private void wobble_servo(){
        if(gamepad2.y.toggle)
            wobble_servo.setPosition(1);
        else{
            wobble_servo.setPosition(0);
        }
    }


    //    FTC-JjOY
//    private void wing(){
//        if (gamepad2.a.toggle){
//            wingLeft.setPosition(0.85);
//            wingRight.setPosition(1);
//        }
//        else {
//            wingLeft.setPosition(0);
//            wingRight.setPosition(0);
//        }
//    }

//    FTC-Chineese


    private void antena(){
        if(gamepad2.right_bumper.toggle){
            antena.setPosition(0.77);
        }
        else {
            antena.setPosition(0);
        }
    }

}