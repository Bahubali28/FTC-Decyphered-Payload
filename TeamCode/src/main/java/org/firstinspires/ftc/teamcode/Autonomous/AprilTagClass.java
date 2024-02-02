package org.firstinspires.ftc.teamcode.Autonomous;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.opMode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.concurrent.TimeUnit;

public class AprilTagClass {

    static AprilTagDetectionPipeline aprilTagDetectionPipeline;
    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    static double fx = 578.272;
    static double fy = 578.272;
    static double cx = 402.145;
    static double cy = 221.506;

    // UNITS ARE METERS
    static double tagsize = 0.166;

    int ID_TAG_OF_INTEREST = 18; // Tag ID 18 from the 36h11 family
    static int CENTER = 5;
    int RIGHT = 6;
    int LEFT = 4;
    static AprilTagDetection tagOfInterest = null;
    static boolean dontre;
    static boolean desYaw;
    static boolean desX;
    static boolean checkYaw;
    static boolean desZ;
    static boolean checkYaw2;

    static boolean parkNeeded;
    static boolean placedPixel = false;
    static int step2;
    private static DcMotor fr;
    private static DcMotor fl;
    private static DcMotor bl;
    private static DcMotor br;
    private static Servo serIn4;
    private static Servo serIn5;
    private static CRServo serIn1;
    private static CRServo serIn2;

    public static void moveForward(double power) {
        fl.setPower(-power);
        fr.setPower(-power);
        bl.setPower(-power);
        br.setPower(-power);
    }
    public static void moveBackward(double power) {
        fl.setPower(power);
        fr.setPower(power);
        bl.setPower(power);
        br.setPower(power);
    }
    public static void turnLeft(double power) {
        fl.setPower(power);
        fr.setPower(-power);
        bl.setPower(power);
        br.setPower(-power);
    }
    public static void turnRight(double power) {
        fl.setPower(-power);
        fr.setPower(power);
        bl.setPower(-power);
        br.setPower(power);
    }
    public static void strafeRight(double power) {
        fl.setPower(power);
        fr.setPower(-power);
        bl.setPower(-power);
        br.setPower(power);
    }
    public static void strafeLeft(double power) {
        fl.setPower(-power);
        fr.setPower(power);
        bl.setPower(power);
        br.setPower(-power);
    }
    public static void Idle() {
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
    }
    public static void initAprilTag(String CameraName) {
        OpenCvCamera camera;
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, CameraName), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);
    }
    public static void moveToAprilTag(Boolean opModeStatus) throws InterruptedException {
        dontre = false;
        desYaw = false;
        checkYaw = false;
        desX = false;
        checkYaw2 = false;
        desZ = false;
        parkNeeded = true;
        step2 = 1;
        if (step2 == 1) {
            telemetry.addLine("Opmode is currently active");
            telemetry.update();
//                try{
//	    /*
//	    1. Move forward to the second bar
//	   2. Move sideways through
//	   the bar to the backdrop for 5 points
//	   */
//                    moveForward(0.5);
//                    TimeUnit.MILLISECONDS.sleep(740);
//                    Idle();
//                    TimeUnit.MILLISECONDS.sleep(700);
//                    telemetry.addLine("Turned right");
//                    telemetry.update();
//                    turnRight(0.5);
//                    TimeUnit.MILLISECONDS.sleep(1050);
//                    moveForward(0.5);
//                    TimeUnit.MILLISECONDS.sleep(2750);
//                    Idle();
//                    telemetry.addLine("Autonomous Setup Complete! :)");
//                    telemetry.update();
//                } catch(InterruptedException e){
////                    TODO: handle exception
//                }
            step2++;
        }
        telemetry.addLine("Moving to the April Tag...");
        telemetry.update();
        ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();
        if (currentDetections.size() != 0)
        {
            boolean tagFound = false;

            for(AprilTagDetection tag : currentDetections)
            {
                if(tag.id == CENTER)
                {
                    tagOfInterest = tag;
                    tagFound = true;
                    break;
                } else if (tag.id != CENTER){
                    tagFound = false;
                }
            }

            if(tagFound)
            {
                double deadzone = 0.1;

                telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                tagToTelemetry(tagOfInterest);
                Orientation rot = Orientation.getOrientation(tagOfInterest.pose.R, AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES);
                do {
                    if (step2 == 2 &&!desYaw) {
                        telemetry.addData("step2: ", step2);
                        //                    telemetry.addData("Tag found: ", tagOfInterest.id);
                        telemetry.update();
                        if (rot.firstAngle < -2) {
                            turnLeft(0.2);
                        } else if (rot.firstAngle > 2) {
                            turnRight(0.2);
                        } else {
                            Idle();
                            desYaw = true;
                            telemetry.addData("desYaw: ", desYaw);
                            telemetry.update();
                            step2++;
                        }
                    }
                    if (step2 == 3 && !desX) {
                        telemetry.addData("step2: ", step2);
                        telemetry.update();
                        if(tagOfInterest.pose.x < -deadzone) {
                            strafeLeft(0.2);
                        }
                        else if(tagOfInterest.pose.x > deadzone) {
                            // do something else
                            strafeRight(0.2);
                        } else {
                            Idle();
                            desX = true;
                            telemetry.addData("desX: ", desX);
                            telemetry.update();
                            step2++;
                        }
                    }
                    if (step2 == 4 && !checkYaw) {
                        telemetry.addData("step2: ", step2);
                        telemetry.update();
                        if (rot.firstAngle < -1) {
                            turnLeft(0.25);
                        } else if (rot.firstAngle > 1) {
                            turnRight(0.25);
                        } else {
                            Idle();
                            checkYaw = true;
                            telemetry.addData("checkYaw: ", checkYaw);
                            telemetry.update();
                            step2++;
                        }
                    }
                    if (step2 == 5 && !desZ) {
                        telemetry.addData("step2: ", step2);
                        telemetry.update();
                        if (tagOfInterest.pose.z > 0.90) {
                            moveForward(0.20 );

                        } else {
                            Idle();
                            desZ = true;
                            telemetry.addData("desZ: ", desZ);
                            telemetry.update();
                            step2++;
                        }
                    }
                    if (step2 == 6 && !checkYaw2) {
                        telemetry.addData("step2: ", step2);
                        telemetry.update();
                        if (rot.firstAngle < -2) {
                            turnLeft(0.1);
                        } else if (rot.firstAngle > 2) {
                            turnRight(0.1);
                        } else {
                            Idle();
                            checkYaw2 = true;
                            telemetry.addData("checkYaw2: ", checkYaw2);
                            telemetry.update();
                            step2++;
                        }
                    }
                    if (desX && desZ) {
                        telemetry.addLine("Reached the april tag.");
                        telemetry.addLine("Placing the pixel");
                        telemetry.update();
                        serIn4.setPosition(0.75);
                        serIn5.setPosition(0.75);
                        TimeUnit.SECONDS.sleep(1);
                        placedPixel = true;
                    }
                    if (placedPixel) {
                        serIn1.setPower(0.5);
                        serIn2.setPower(-0.5);
                        TimeUnit.MILLISECONDS.sleep(500);
                        serIn1.setPower(0);
                        serIn2.setPower(0);
                        TimeUnit.SECONDS.sleep(1);
                        serIn4.setPosition(0);
                        serIn5.setPosition(0);
                        step2++;
                        moveBackward(0.3);
                        TimeUnit.MILLISECONDS.sleep(300);
                        Idle();
                        TimeUnit.MILLISECONDS.sleep(300);
                        turnRight(0.5);
                        TimeUnit.MILLISECONDS.sleep(845);
                        Idle();
                        moveForward(0.5);
                            /*Later, this distance needs to be altered
                            because the distance between the april tags
                            makes the distance to park longer or shorter.
                             */
                        TimeUnit.MILLISECONDS.sleep(820);
                        Idle();
                        turnLeft(0.5);
                        TimeUnit.MILLISECONDS.sleep(700);
                        Idle();
                            /*
                            moveForward(0.5);
                            TimeUnit.MILLISECONDS.sleep(500);
                            */
                        Idle();
                        telemetry.addLine("-------------------------------");
                        telemetry.addLine("Autonomous complete.");
                        telemetry.addLine("-------------------------------");
                        telemetry.update();
                        Idle();
                    }
                } while (step2 == 9 && desX && checkYaw && desZ && checkYaw2 && opModeStatus && parkNeeded);
                    /*
                        if (step2 == 66) {
                            strafeRight(0.5);
                            TimeUnit.MILLISECONDS.sleep(250);
                            Idle();
                            telemetry.addLine("Strafed Left");
                            telemetry.addLine("Ready to park.");
                            telemetry.update();
                            moveForward(0.5);
                            TimeUnit.MILLISECONDS.sleep(250);
                            Idle();
                            telemetry.addLine("Parked");
                            telemetry.update();
                            parkNeeded = false;
                        } else if (!parkNeeded) {
                            step2++;
                        }
                        */

            }
            else
            {
                Idle();
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }
            }

            telemetry.update();
            sleep(20);
        }
        else
        {
            Idle();
            telemetry.addLine("Don't see tag of interest :(");

            if(tagOfInterest == null)
            {
                telemetry.addLine("(The tag has never been seen)");
            }
            else
            {
                telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                tagToTelemetry(tagOfInterest);
            }

        }
    }

    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Initializing...");
        telemetry.update();
        //hardwareMap.
        fl = hardwareMap.get(DcMotor.class, "fl");
        br = hardwareMap.get(DcMotor.class, "br");
        fr = hardwareMap.get(DcMotor.class, "fr");
        bl = hardwareMap.get(DcMotor.class, "bl");
        serIn4 = hardwareMap.get(Servo.class, "serIn4");
        serIn5 = hardwareMap.get(Servo.class, "serIn5");
        serIn1 = hardwareMap.get(CRServo.class, "serIn1");
        serIn2 = hardwareMap.get(CRServo.class, "serIn2");

        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        serIn1.setDirection(CRServo.Direction.REVERSE);
        serIn2.setDirection(CRServo.Direction.REVERSE);
        serIn5.setDirection(Servo.Direction.REVERSE);
        serIn4.setDirection(Servo.Direction.FORWARD);
        serIn4.setPosition(0);
        serIn5.setPosition(0);


//        turnLeft(0.5);
//        TimeUnit.MILLISECONDS.sleep(1000);

        telemetry.addLine("Autonomous_RED_BACK Initialized...");
        telemetry.update();
        telemetry.addLine("Everything Works");
        telemetry.addLine("Ready to Start Autonomous");
        telemetry.update();
        //       waitForStart();
    }

    static void tagToTelemetry(AprilTagDetection detection)
    {
        Orientation rot = Orientation.getOrientation(detection.pose.R, AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES);

        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", rot.firstAngle));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", rot.secondAngle));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", rot.thirdAngle));
    }
}
