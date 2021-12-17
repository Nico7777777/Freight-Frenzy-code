//package org.firstinspires.ftc.teamcode.drive.advanced;
//
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.geometry.Vector2d;
//import com.acmerobotics.roadrunner.trajectory.Trajectory;
//import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
//import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
//import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
//import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.teamcode.drive.DriveConstants;
//import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
//import org.firstinspires.ftc.teamcode.drive.opmode.OpenCV;
//import org.opencv.core.Core;
//import org.opencv.core.Mat;
//import org.opencv.core.Point;
//import org.opencv.core.Rect;
//import org.opencv.core.Scalar;
//import org.opencv.imgproc.Imgproc;
//import org.openftc.easyopencv.OpenCvCamera;
//import org.openftc.easyopencv.OpenCvCameraFactory;
//import org.openftc.easyopencv.OpenCvCameraRotation;
//import org.openftc.easyopencv.OpenCvPipeline;
//
//import java.util.Arrays;
//
//import static org.openftc.easyopencv.OpenCvCameraFactory.*;
//
//@Autonomous
//public class AutoBlue133_138 extends LinearOpMode {
//
//    //drivetrain
//    SampleMecanumDrive drive;
//
//    //servo
//    Servo manuta,wobbleServo,antena;
//
//    //motoare
//    DcMotor shooter1,shooter2,wobble,intake;
//
//    //opencv
//    private OpenCvCamera webcam;
//    private UltimateGoalPipeline pipeline;
//
//    //opmode
//    @Override
//    public void runOpMode() throws InterruptedException {
//
//        //init
//        initHardware();
//        initOpenCv();
//        while(!opModeIsActive() && !isStopRequested()){
//            telemetry.addData("case: " , pipeline.getAnalysis());
//            telemetry.update();
//        }
//        waitForStart();
//
//        if(isStopRequested()) return;
//
//        makeCase();
//    }
//
//    //alege cazul
//    private void makeCase() {
//
//        if(pipeline.getAnalysis() == UltimateGoalPipeline.UltimateGoalRings.FOUR)
//            cazulFOUR();
//
//        else if(pipeline.getAnalysis() == UltimateGoalPipeline.UltimateGoalRings.ONE)
//            cazulONE();
//
//        else if(pipeline.getAnalysis() == UltimateGoalPipeline.UltimateGoalRings.ZERO)
//            cazulZERO();
//    }
//
//
//
//    //cazuri
//    private void cazulZERO(){
////        Trajectory powerShot = drive.trajectoryBuilder(new Pose2d(0,0,0))
////                .strafeTo(new Vector2d(60,-20))
////                .addTemporalMarker(0.2,() ->{
////                    antena.setPosition(0);
////                })
////                .build();
//
//        Trajectory shootHigh = drive.trajectoryBuilder(new Pose2d(0,0,0))
//                .strafeTo(new Vector2d(60,0))
//                .build();
//
//        Trajectory dropWobble = drive.trajectoryBuilder(new Pose2d(60,0,0))
//                .strafeTo(new Vector2d(64,33))
//                .build();
//
//        Trajectory getWobble = drive.trajectoryBuilder(new Pose2d(64,33,0))
//                .strafeTo(new Vector2d(32.5,25))
//                .addTemporalMarker(0.4,() ->{
//                    wobble.setPower(-1);
//                })
//                .build();
//
//        Trajectory dropSecondWobble = drive.trajectoryBuilder(new Pose2d(32.5,25,0))
//                .lineToLinearHeading(new Pose2d(74,17,Math.toRadians(-90)))
//                .build();
//
//        Trajectory park = drive.trajectoryBuilder(new Pose2d(74,17,Math.toRadians(-90)))
//                .strafeTo(new Vector2d(73,10))
//                .build();
//
//
//
//
//        shooter(0.71);
//        antena.setPosition(1);
//        drive.followTrajectory(shootHigh);
//        antena.setPosition(0);
//        shootHigh();
//        drive.followTrajectory(dropWobble);
//        wobbleServo.setPosition(0.7);
//        drive.followTrajectory(getWobble);
//        sleep(300);
//        wobbleServo.setPosition(0);
//        sleep(300);
//        drive.followTrajectory(dropSecondWobble);
//        wobbleServo.setPosition(0.7);
//        drive.followTrajectory(park);
//
//    }
//    private void cazulONE(){
//
//        Trajectory shootHigh = drive.trajectoryBuilder(new Pose2d(0,0,0))
//                .strafeTo(new Vector2d(60,0))
//                .build();
//
//
//        Trajectory dropWobble = drive.trajectoryBuilder(new Pose2d(60,0))
//                .strafeTo(new Vector2d(86,10))
//                .build();
//
//        Trajectory prepareForWobble = drive.trajectoryBuilder(new Pose2d(86,10))
//                .strafeTo(new Vector2d(80,30))
//                .build();
//
//        Trajectory getWobble = drive.trajectoryBuilder(new Pose2d(80,30))
//                .strafeTo(new Vector2d(32,25))
//                .addTemporalMarker(0.5,() ->{
//                    wobble.setPower(-1);
//                })
//                .build();
//
//        Trajectory prepareForRing = drive.trajectoryBuilder(new Pose2d(32,25))
//                .strafeTo(new Vector2d(17,15))
//                .build();
//
//
//        Trajectory shootRing  = drive.trajectoryBuilder(new Pose2d(17,15))
//                .lineToLinearHeading(new Pose2d(58,15,Math.toRadians(-10)),
//                        new MinVelocityConstraint(
//                                Arrays.asList(
//                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
//                                        new MecanumVelocityConstraint(40 ,DriveConstants.TRACK_WIDTH)
//                                )
//                        ),new ProfileAccelerationConstraint(10)
//                )
//                .addTemporalMarker(0.01,() ->{
//                    shooter(0.82);
//                })
//                .addTemporalMarker(0.5,() ->{
//                    wobble.setPower(-1);
//                })
//                .build();
//
//
//        Trajectory dropSecondWobble = drive.trajectoryBuilder(new Pose2d(58,15,0))
//                .lineToLinearHeading(new Pose2d(80,11,Math.toRadians(-180)))
//                .build();
//
//        Trajectory park = drive.trajectoryBuilder(new Pose2d(80,11,Math.toRadians(-180)))
//                .strafeTo(new Vector2d(75,11))
//                .build();
//
//
//
//
//        shooter(0.71);
//        antena.setPosition(1);
//        drive.followTrajectory(shootHigh);
//        antena.setPosition(0);
//        shootHigh();
//        drive.followTrajectory(dropWobble);
//        wobbleServo.setPosition(0.72);
//        drive.followTrajectory(prepareForWobble);
//        drive.followTrajectory(getWobble);
//        wobbleServo.setPosition(0);
//        sleep(300);
//        drive.followTrajectory(prepareForRing);
//        wobble.setPower(0);
//        intake.setPower(1);
//        drive.turn(Math.toRadians(-10));
//        drive.followTrajectory(shootRing);
//        sleep(2000);
//        manuta.setPosition(0.5);
//        sleep(100);
//        manuta.setPosition(0.1);
//        sleep(400);
//        shooter(0);
//        drive.followTrajectory(dropSecondWobble);
//        wobbleServo.setPosition(0.7);
//        drive.followTrajectory(park);
//    }
//    private void cazulFOUR(){
//
//        Trajectory shootHigh = drive.trajectoryBuilder(new Pose2d(0,0,0))
//                .strafeTo(new Vector2d(60,-1))
//                .build();
//
//
//        Trajectory dropWobble = drive.trajectoryBuilder(new Pose2d(60,-1))
//                .strafeTo(new Vector2d(117,33),
//                        new MinVelocityConstraint(
//                                Arrays.asList(
//                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
//                                        new MecanumVelocityConstraint(100 ,DriveConstants.TRACK_WIDTH)
//                                )
//                        ),new ProfileAccelerationConstraint(100)
//                )
//                .build();
//
//
//        Trajectory getWobble = drive.trajectoryBuilder(new Pose2d(117,33))
//                .strafeTo(new Vector2d(33.7,21.7),
//                        new MinVelocityConstraint(
//                                Arrays.asList(
//                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
//                                        new MecanumVelocityConstraint(100 ,DriveConstants.TRACK_WIDTH)
//                                )
//                        ),new ProfileAccelerationConstraint(40)
//                )
//                .addTemporalMarker(0.5,() ->{
//                    wobble.setPower(-1);
//                })
//                .build();
//
//        Trajectory prepareForRing1 = drive.trajectoryBuilder(new Pose2d(33.7,21.7))
//                .strafeTo(new Vector2d(24,20),
//                        new MinVelocityConstraint(
//                                Arrays.asList(
//                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
//                                        new MecanumVelocityConstraint(100 ,DriveConstants.TRACK_WIDTH)
//                                )
//                        ),new ProfileAccelerationConstraint(30)
//                )
//                .build();
//
//        Trajectory prepareForRing2 = drive.trajectoryBuilder(new Pose2d(24,20))
//                .strafeTo(new Vector2d(24,12),
//                        new MinVelocityConstraint(
//                                Arrays.asList(
//                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
//                                        new MecanumVelocityConstraint(100 ,DriveConstants.TRACK_WIDTH)
//                                )
//                        ),new ProfileAccelerationConstraint(30)
//                )
//                .build();
//
//
//        Trajectory shootRing  = drive.trajectoryBuilder(new Pose2d(24,12))
//                .strafeTo(new Vector2d(41,12),
//                        new MinVelocityConstraint(
//                                Arrays.asList(
//                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
//                                        new MecanumVelocityConstraint(7 ,DriveConstants.TRACK_WIDTH)
//                                )
//                        ),new ProfileAccelerationConstraint(10 )
//                )
//                .addTemporalMarker(0.04,() ->{
//                    shooter(0.88);
//                })
//                .build();
//
//        Trajectory getLastRing = drive.trajectoryBuilder(new Pose2d(41,12))
//                .strafeTo(new Vector2d(60,12),
//                        new MinVelocityConstraint(
//                                Arrays.asList(
//                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
//                                        new MecanumVelocityConstraint(60 ,DriveConstants.TRACK_WIDTH)
//                                )
//                        ),new ProfileAccelerationConstraint(30)
//                )
//                .addTemporalMarker(0.01,() ->{
//                    shooter((0.84));
//                })
//                .build();
//
//        Trajectory dropSecondWobble = drive.trajectoryBuilder(new Pose2d(60,12,-165))
//                .lineToLinearHeading(new Pose2d(111,30,Math.toRadians(-190)),
//                        new MinVelocityConstraint(
//                                Arrays.asList(
//                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
//                                        new MecanumVelocityConstraint(100,DriveConstants.TRACK_WIDTH)
//                                )
//                        ),new ProfileAccelerationConstraint(100)
//                )
//                .build();
//
//
//        Trajectory park = drive.trajectoryBuilder(new Pose2d(111,30,Math.toRadians(-190)))
//                .strafeTo(new Vector2d(90,30),
//                        new MinVelocityConstraint(
//                                Arrays.asList(
//                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
//                                        new MecanumVelocityConstraint(100 ,DriveConstants.TRACK_WIDTH)
//                                )
//                        ),new ProfileAccelerationConstraint(100)
//                )
//                .build();
//
//
//
//
//        shooter(0.71);
//        antena.setPosition(1);
//        drive.followTrajectory(shootHigh);
//        antena.setPosition(0);
//        shootHigh();
//        drive.followTrajectory(dropWobble);
//        wobbleServo.setPosition(0.72);
//        drive.followTrajectory(getWobble);
//        wobbleServo.setPosition(0);
//        sleep(300);
//        intake.setPower(1);
//        drive.followTrajectory(prepareForRing1);
//        drive.followTrajectory(prepareForRing2);
//        drive.followTrajectory(shootRing);
//        drive.turn(Math.toRadians(-8));
//        sleep(1000);
//        //1
//        manuta.setPosition(0.5);
//        sleep(100);
//        manuta.setPosition(0);
//        sleep(800);
//        //2
//        manuta.setPosition(0.5);
//        sleep(100);
//        manuta.setPosition(0);
//        sleep(800);
//        //3
//        manuta.setPosition(0.5);
//        sleep(100);
//        manuta.setPosition(0);
//        sleep(200);
//        drive.turn(Math.toRadians(8.5));
//
//        drive.followTrajectory(getLastRing);
//
//        drive.turn(Math.toRadians(-10));
//        sleep(1000);
//        //1
//        manuta.setPosition(0.5);
//        sleep(100);
//        manuta.setPosition(0);
//        sleep(500);
//        //2
//        manuta.setPosition(0.5);
//        sleep(100);
//        manuta.setPosition(0);
//        sleep(300);
//        //3
//        manuta.setPosition(0.5);
//        sleep(100);
//        manuta.setPosition(0);
//        sleep(200);
//        drive.turn(Math.toRadians(-155));
//
//
//        drive.followTrajectory(dropSecondWobble);
//        shooter(0);
//        sleep(100);
//        wobbleServo.setPosition(1);
//        drive.followTrajectory(park);
//        wobbleServo.setPosition(0);
//
//    }
//
//
//    //shooter
//    private void flickerSequenceZERO(){
//        //1
//        drive.turn(Math.toRadians(1));
//        sleep(100);
//        manuta.setPosition(0.5);
//        sleep(100);
//        manuta.setPosition(0.1);
//        sleep(300);
//        //2
//        drive.turn(Math.toRadians(-6));
//        sleep(100);
//        manuta.setPosition(0.5);
//        sleep(100);
//        manuta.setPosition(0.1);
//        sleep(300);
//        //3
//        drive.turn(Math.toRadians(-5));
//        sleep(100);
//        manuta.setPosition(0.5);
//        sleep(100);
//        manuta.setPosition(0.1);
//        sleep(300);
//        shooter(0);
//        drive.turn(Math.toRadians(10));
//    }
//    private void flickerSequenceONE(){
//        //1
//        drive.turn(Math.toRadians(-12));
//        sleep(100);
//        manuta.setPosition(0.5);
//        sleep(100);
//        manuta.setPosition(0.1);
//        sleep(300);
//        //2
//        drive.turn(Math.toRadians(-8));
//        sleep(100);
//        manuta.setPosition(0.5);
//        sleep(100);
//        manuta.setPosition(0.1);
//        sleep(300);
//        //3
//        drive.turn(Math.toRadians(-4));
//        sleep(100);
//        manuta.setPosition(0.5);
//        sleep(100);
//        manuta.setPosition(0.1);
//        drive.turn(Math.toRadians(24));
//        shooter(0);
//    }
//    private void flickerSequenceFOUR(){
//        //1
//        drive.turn(Math.toRadians(2));
//        sleep(100);
//        manuta.setPosition(0.7);
//        sleep(100);
//        manuta.setPosition(0);
//        sleep(700);
//        //2
//        drive.turn(Math.toRadians(-6));
//        sleep(110);
//        manuta.setPosition(0.7);
//        sleep(100);
//        manuta.setPosition(0);
//        sleep(700);
//        //3
//        drive.turn(Math.toRadians(-7));
//        sleep(110);
//        manuta.setPosition(0.7);
//        sleep(100);
//        manuta.setPosition(0);
//        shooter(0);
//        drive.turn(Math.toRadians(11));
//    }
//
//    private void shootHigh(){
////        drive.turn(Math.toRadians(0.5));
//        sleep(100);
//        manuta.setPosition(0.5);
//        sleep(100);
//        manuta.setPosition(0.1);
//        sleep(600);
//        //2
//        manuta.setPosition(0.5);
//        sleep(100);
//        manuta.setPosition(0.1);
//        sleep(600);
//        //3
//        manuta.setPosition(0.5);
//        sleep(100);
//        manuta.setPosition(0.1);
//        sleep(600);
//        shooter(0);
////        drive.turn(Math.toRadians(-0.5));
//    }
//
//    //inituri
//    private void initOpenCv(){
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        webcam =  getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"),cameraMonitorViewId);
//        pipeline = new UltimateGoalPipeline(telemetry);
//        webcam.setPipeline(pipeline);
//        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
//            @Override
//            public void onOpened() {
//                webcam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
//            }
//        });
//    }
//    private void initHardware(){
//
//        drive = new SampleMecanumDrive(hardwareMap);
//        manuta = hardwareMap.get(Servo.class,"servo");
//        antena = hardwareMap.get(Servo.class,"antena");
//        wobbleServo = hardwareMap.get(Servo.class, "wobble_servo");
//        wobble= hardwareMap.get(DcMotor.class, "wobble");
//        shooter1 = hardwareMap.get(DcMotor.class,"shooter1");
//        shooter2 = hardwareMap.get(DcMotor.class,"shooter2");
//        intake = hardwareMap.get(DcMotor.class,"intake");
//
//
//
//        wobble.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        antena.setDirection(Servo.Direction.REVERSE);
////        wobbleServo.setDirection(Servo.Direction.REVERSE);
//
//
//        wobbleServo.setPosition(0);
//        manuta.setPosition(0);
//    }
//
//
//    //shooter
//    private void shooter(double speed){
//        shooter1.setPower(speed);
//        shooter2.setPower(speed);
//    }
//    private void shootSequence() {
//        //1
//        sleep(100);
//        manuta.setPosition(0.7);
//        sleep(100);
//        manuta.setPosition(0);
//        sleep(100);
//        //2
//
//        sleep(150);
//        manuta.setPosition(0.7);
//        sleep(100);
//        manuta.setPosition(0);
//        sleep(100);
//        //3
//        sleep(150);
//        manuta.setPosition(0.7);
//        sleep(100);
//        manuta.setPosition(0);
//        sleep(100);
//    }
//
//    //clasa openCv
//    public static class UltimateGoalPipeline extends OpenCvPipeline
//    {
//        Telemetry telemetry;
//
//        public UltimateGoalPipeline(Telemetry telemetry){
//            this.telemetry = telemetry;
//
//        }
//
//        public enum UltimateGoalRings
//        {
//            FOUR,
//            ONE,
//            ZERO
//        }
//
//
//        static final Scalar RED = new Scalar(255, 0, 0);
//
//        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(461,531);
//
//        static final int REGION_WIDTH = 120;
//        static final int REGION_HEIGHT = 80;
//        static final int FOUR_RING_THRESHOLD = 105; // 93    124 for ZERO
//        static final int ONE_RING_THRESHOLD = 115; //104
//
//
//        Point region1_pointA = new Point(
//                REGION1_TOPLEFT_ANCHOR_POINT.x,
//                REGION1_TOPLEFT_ANCHOR_POINT.y);
//        Point region1_pointB = new Point(
//                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
//                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
//
//
//        Mat region1_Cb;
//        Mat YCrCb = new Mat();
//        Mat Cb = new Mat();
//        int avg;
//
//        private volatile UltimateGoalRings position = UltimateGoalRings.ZERO;
//
//        void inputToCb(Mat input)
//        {
//            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb );
//            Core.extractChannel(YCrCb, Cb, 2);
//        }
//
//        @Override
//        public void init(Mat firstFrame)
//        {
//            inputToCb(firstFrame);
//            region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
//        }
//
//        @Override
//        public Mat processFrame(Mat input)
//        {
//            inputToCb(input);
//
//            avg = (int) Core.mean(region1_Cb).val[0];
//            telemetry.addData("avg: ",avg);
//            Imgproc.rectangle(
//                    input, // Buffer to draw on
//                    region1_pointA, // First point which defines the rectangle
//                    region1_pointB, // Second point which defines the rectangle
//                    RED, // The color the rectangle is drawn in
//                    2); // Thickness of the rectangle lines
//
//
//            if(avg <= FOUR_RING_THRESHOLD)
//            {
//                position = UltimateGoalRings.FOUR;
//
//            }
//            else if(avg <= ONE_RING_THRESHOLD)
//            {
//                position = UltimateGoalRings.ONE;
//
//            }
//            else
//            {
//                position = UltimateGoalRings.ZERO;
//            }
//
//            return input;
//        }
//
//        public UltimateGoalRings getAnalysis()
//        {
//            return position;
//        }
//    }
//}