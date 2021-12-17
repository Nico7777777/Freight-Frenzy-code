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
//import org.apache.commons.math3.analysis.function.Pow;
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
//public class AutoRedComuna_Special extends LinearOpMode {
//
//    //drivetrain
//    SampleMecanumDrive drive;
//
//    //servo
//    Servo manuta,wobbleServo;
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
////        initOpenCv();
////        while(!opModeIsActive() && !isStopRequested()){
////            telemetry.addData("case: " , pipeline.getAnalysis());
////            telemetry.update();
////        }
//        waitForStart();
//
//        if(isStopRequested()) return;
//
////        makeCase();
//        cazulZERO();
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
//
//        Trajectory shootHigh = drive.trajectoryBuilder(new Pose2d(0, 0, 0))
//                .strafeTo(new Vector2d(60, 0),
//                        new MinVelocityConstraint(
//                                Arrays.asList(
//                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
//                                        new MecanumVelocityConstraint(60, DriveConstants.TRACK_WIDTH)
//                                )
//                        ), new ProfileAccelerationConstraint(40)
//                )
//                .build();
//
//        Trajectory prepareBounceBack1 = drive.trajectoryBuilder(new Pose2d(60, 0, 0))
//                .strafeTo(new Vector2d(50, 16),
//                        new MinVelocityConstraint(
//                                Arrays.asList(
//                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
//                                        new MecanumVelocityConstraint(60, DriveConstants.TRACK_WIDTH)
//                                )
//                        ), new ProfileAccelerationConstraint(40)
//                )
//                .build();
//
//        Trajectory prepareBounceBack2 = drive.trajectoryBuilder(new Pose2d(50, 16, 0))
//                .strafeTo(new Vector2d(118, 16),
//                        new MinVelocityConstraint(
//                                Arrays.asList(
//                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
//                                        new MecanumVelocityConstraint(60, DriveConstants.TRACK_WIDTH)
//                                )
//                        ), new ProfileAccelerationConstraint(40)
//                )
//                .build();
//
//        Trajectory bounceBack = drive.trajectoryBuilder(new Pose2d(118, 16, 90))
//                .lineToLinearHeading(new Pose2d(118, 60, Math.toRadians(90)),
//                        new MinVelocityConstraint(
//                                Arrays.asList(
//                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
//                                        new MecanumVelocityConstraint(60, DriveConstants.TRACK_WIDTH)
//                                )
//                        ), new ProfileAccelerationConstraint(40)
//                )
//                .build();
//
//        Trajectory shootBounceBack = drive.trajectoryBuilder(new Pose2d(118, 60, 90))
//                .lineToLinearHeading(new Pose2d(60, 20, Math.toRadians(0)),
//                        new MinVelocityConstraint(
//                                Arrays.asList(
//                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
//                                        new MecanumVelocityConstraint(60, DriveConstants.TRACK_WIDTH)
//                                )
//                        ), new ProfileAccelerationConstraint(40)
//                )
//                .addTemporalMarker(0.5, () -> {
//                    shooter(0.92);
//                })
//                .build();
//
//
//
//
//
//
//
//
//        shooter(-0.92);
//        drive.followTrajectory(shootHigh);
//        drive.turn(Math.toRadians(25));
//
//        //1
//        sleep(600);
//        manuta.setPosition(0.55);
//        sleep(200);
//        manuta.setPosition(0);
//        sleep(200);
//        //2
//        manuta.setPosition(0.55);
//        sleep(200);
//        manuta.setPosition(0);
//        sleep(200);
//        //3
//        manuta.setPosition(0.55);
//        sleep(200);
//        manuta.setPosition(0);
//
//        sleep(200);
//        drive.turn(Math.toRadians(-25));
//        shooter(0);
//
//        sleep(200);
//        wobbleServo.setPosition(0.72);
//        sleep(200);
//
//        drive.followTrajectory(prepareBounceBack1);
//        intake.setPower(-1);
//        drive.followTrajectory(prepareBounceBack2);
//        drive.turn(Math.toRadians(90));
//        drive.followTrajectory(bounceBack);
//        drive.followTrajectory(shootBounceBack);
//
//
//        //1
//        sleep(600);
//        manuta.setPosition(0.55);
//        sleep(200);
//        manuta.setPosition(0);
//        sleep(200);
//        //2
//        manuta.setPosition(0.55);
//        sleep(200);
//        manuta.setPosition(0);
//        sleep(200);
//        //3
//        manuta.setPosition(0.55);
//        sleep(200);
//        manuta.setPosition(0);
//
//        sleep(200);
//
//
//
//    }
//    private void cazulONE(){
//
//
//        Trajectory shootHigh = drive.trajectoryBuilder(new Pose2d(0, 0, 0))
//                .strafeTo(new Vector2d(60, 0),
//                        new MinVelocityConstraint(
//                                Arrays.asList(
//                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
//                                        new MecanumVelocityConstraint(60, DriveConstants.TRACK_WIDTH)
//                                )
//                        ), new ProfileAccelerationConstraint(40)
//                )
//                .build();
//
//        Trajectory dropWobble = drive.trajectoryBuilder(new Pose2d(60, 0, 0))
//                .strafeTo(new Vector2d(90, 20),
//                        new MinVelocityConstraint(
//                                Arrays.asList(
//                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
//                                        new MecanumVelocityConstraint(60, DriveConstants.TRACK_WIDTH)
//                                )
//                        ), new ProfileAccelerationConstraint(40)
//                )
//                .build();
//
//        Trajectory prepareForRings1 = drive.trajectoryBuilder(new Pose2d(90, 20, 0))
//                .strafeTo(new Vector2d(60, 0),
//                        new MinVelocityConstraint(
//                                Arrays.asList(
//                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
//                                        new MecanumVelocityConstraint(60, DriveConstants.TRACK_WIDTH)
//                                )
//                        ), new ProfileAccelerationConstraint(40)
//                )
//                .build();
//
//        Trajectory prepareForRings2 = drive.trajectoryBuilder(new Pose2d(60, 0, 0))
//                .strafeTo(new Vector2d(20, 0),
//                        new MinVelocityConstraint(
//                                Arrays.asList(
//                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
//                                        new MecanumVelocityConstraint(60, DriveConstants.TRACK_WIDTH)
//                                )
//                        ), new ProfileAccelerationConstraint(40)
//                )
//                .build();
//
//        Trajectory prepareForRings3 = drive.trajectoryBuilder(new Pose2d(20, 0, 0))
//                .strafeTo(new Vector2d(14, 24),
//                        new MinVelocityConstraint(
//                                Arrays.asList(
//                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
//                                        new MecanumVelocityConstraint(60, DriveConstants.TRACK_WIDTH)
//                                )
//                        ), new ProfileAccelerationConstraint(40)
//                )
//                .build();
//
//        Trajectory pushRings = drive.trajectoryBuilder(new Pose2d(14, 24))
//                .lineToLinearHeading(new Pose2d(37, 24, Math.toRadians(10)),
//                        new MinVelocityConstraint(
//                                Arrays.asList(
//                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
//                                        new MecanumVelocityConstraint(100, DriveConstants.TRACK_WIDTH)
//                                )
//                        ), new ProfileAccelerationConstraint(100)
//                )
//                .build();
//
//
//
//
//
//        shooter(-0.92);
//        drive.followTrajectory(shootHigh);
//        drive.turn(Math.toRadians(25));
//
//        //1
//        sleep(600);
//        manuta.setPosition(0.55);
//        sleep(200);
//        manuta.setPosition(0);
//        sleep(200);
//        //2
//        manuta.setPosition(0.55);
//        sleep(200);
//        manuta.setPosition(0);
//        sleep(200);
//        //3
//        manuta.setPosition(0.55);
//        sleep(200);
//        manuta.setPosition(0);
//
//        sleep(200);
//        drive.turn(Math.toRadians(-25));
//        shooter(0);
//
//        drive.followTrajectory(dropWobble);
//
//        wobbleServo.setPosition(0.72);
//        sleep(100);
//
//        drive.followTrajectory(prepareForRings1);
//        drive.followTrajectory(prepareForRings2);
//        drive.followTrajectory(prepareForRings3);
//
//        intake.setPower(-1);
//        shooter(-0.96);
//        drive.followTrajectory(pushRings);
//
//        sleep(1500);
//        manuta.setPosition(0.55);
//        sleep(200);
//        manuta.setPosition(0);
//        sleep(500);
//        shooter(0);
//
//
//
//
//
//
//
//
//
//    }
//    private void cazulFOUR() {
//
//        Trajectory shootHigh = drive.trajectoryBuilder(new Pose2d(0, 0, 0))
//                .strafeTo(new Vector2d(60, 0),
//                        new MinVelocityConstraint(
//                                Arrays.asList(
//                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
//                                        new MecanumVelocityConstraint(60, DriveConstants.TRACK_WIDTH)
//                                )
//                        ), new ProfileAccelerationConstraint(40)
//                )
//                .build();
//
//
//        Trajectory dropWobble = drive.trajectoryBuilder(new Pose2d(60, 0))
//                .strafeTo(new Vector2d(110, 0),
//                        new MinVelocityConstraint(
//                                Arrays.asList(
//                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
//                                        new MecanumVelocityConstraint(100, DriveConstants.TRACK_WIDTH)
//                                )
//                        ), new ProfileAccelerationConstraint(100)
//                )
//                .build();
//
//
//        Trajectory prepareRings1 = drive.trajectoryBuilder(new Pose2d(110, 0))
//                .strafeTo(new Vector2d(20, 0),
//                        new MinVelocityConstraint(
//                                Arrays.asList(
//                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
//                                        new MecanumVelocityConstraint(100, DriveConstants.TRACK_WIDTH)
//                                )
//                        ), new ProfileAccelerationConstraint(100)
//                )
//                .build();
//
//        Trajectory prepareRings2 = drive.trajectoryBuilder(new Pose2d(20, 0))
//                .strafeTo(new Vector2d(14, 30),
//                        new MinVelocityConstraint(
//                                Arrays.asList(
//                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
//                                        new MecanumVelocityConstraint(60, DriveConstants.TRACK_WIDTH)
//                                )
//                        ), new ProfileAccelerationConstraint(40)
//                )
//                .build();
//
//
//
//        Trajectory pushRings = drive.trajectoryBuilder(new Pose2d(14, 30))
//                .lineToLinearHeading(new Pose2d(35.5, 30, Math.toRadians(10)),
//                        new MinVelocityConstraint(
//                                Arrays.asList(
//                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
//                                        new MecanumVelocityConstraint(100, DriveConstants.TRACK_WIDTH)
//                                )
//                        ), new ProfileAccelerationConstraint(100)
//                )
//                .build();
//
//        Trajectory getRings = drive.trajectoryBuilder(new Pose2d(35.5, 30, Math.toRadians(10)))
//                .lineToLinearHeading(new Pose2d(44, 30, Math.toRadians(10)),
//                        new MinVelocityConstraint(
//                                Arrays.asList(
//                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
//                                        new MecanumVelocityConstraint(8, DriveConstants.TRACK_WIDTH)
//                                )
//                        ), new ProfileAccelerationConstraint(10)
//                )
//                .build();
//
//
//
//
//        shooter(-0.92);
//        drive.followTrajectory(shootHigh);
//
//        drive.turn(Math.toRadians(25));
//
//        //1
//        sleep(600);
//        manuta.setPosition(0.55);
//        sleep(200);
//        manuta.setPosition(0);
//        sleep(200);
//        //2
//        manuta.setPosition(0.55);
//        sleep(200);
//        manuta.setPosition(0);
//        sleep(200);
//        //3
//        manuta.setPosition(0.55);
//        sleep(200);
//        manuta.setPosition(0);
//
//        sleep(200);
//        drive.turn(Math.toRadians(-25));
//        shooter(0);
//
//        drive.followTrajectory(dropWobble);
//        sleep(200);
//        wobbleServo.setPosition(0.72);
//
//
//
//
//        drive.followTrajectory(prepareRings1);
//        drive.followTrajectory(prepareRings2);
//        intake.setPower(-1);
//        shooter(-0.95);
//        drive.followTrajectory(pushRings);
//
//        sleep(400);
//        manuta.setPosition(0.55);
//        sleep(200);
//        manuta.setPosition(0);
//
//        shooter(-0.95);
//
//        drive.followTrajectory(getRings);
//
//        //1
//        sleep(300);
//        manuta.setPosition(0.55);
//        sleep(200);
//        manuta.setPosition(0);
//        //2
//        sleep(300);
//        manuta.setPosition(0.55);
//        sleep(200);
//        manuta.setPosition(0);
//        //3
//        sleep(300);
//        manuta.setPosition(0.55);
//        sleep(200);
//        manuta.setPosition(0);
//        sleep(2000);
//
//        shooter(0);
//        intake.setPower(0);
//
//
//
//    }
//
//
//    //inituri
////    private void initOpenCv(){
////        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
////        webcam =  getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"),cameraMonitorViewId);
////        pipeline = new UltimateGoalPipeline(telemetry);
////        webcam.setPipeline(pipeline);
////        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
////            @Override
////            public void onOpened() {
////                webcam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
////            }
////        });
////    }
//    private void initHardware(){
//
//        drive = new SampleMecanumDrive(hardwareMap);
//        manuta = hardwareMap.get(Servo.class,"servo");
//
//
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
//
//        manuta.setDirection(Servo.Direction.REVERSE);
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
//
//
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