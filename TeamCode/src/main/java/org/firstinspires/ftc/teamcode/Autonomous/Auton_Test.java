package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

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

@Autonomous(name = "Autonomous_Test", group = "Auton_Test", preselectTeleOp = "Payload_TeleOp")
public class Auton_Test extends LinearOpMode
{
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    int ID_TAG_OF_INTEREST = 18; // Tag ID 18 from the 36h11 family
    int RIGHT = 6;
    AprilTagDetection tagOfInterest = null;
    boolean dontre;
    boolean desYaw;
    boolean desX;
    boolean checkYaw;
    boolean desZ;
    boolean checkYaw2;
    boolean parkNeeded;
    boolean placedPixel = false;
    int step;
    private DcMotor fr, fl, bl, br;
    private Servo serIn4, serIn5;
    private CRServo serIn1, serIn2;

    public void moveForward(double power) {
        fl.setPower(-power);
        fr.setPower(-power);
        bl.setPower(-power);
        br.setPower(-power);
    }
    public void moveBackward(double power) {
        fl.setPower(power);
        fr.setPower(power);
        bl.setPower(power);
        br.setPower(power);
    }
    public void turnLeft(double power) {
        fl.setPower(power);
        fr.setPower(-power);
        bl.setPower(power);
        br.setPower(-power);
    }
    public void turnRight(double power) {
        fl.setPower(-power);
        fr.setPower(power);
        bl.setPower(-power);
        br.setPower(power);
    }
    public void strafeRight(double power) {
        fl.setPower(power);
        fr.setPower(-power);
        bl.setPower(-power);
        br.setPower(power);
    }
    public void strafeLeft(double power) {
        fl.setPower(-power);
        fr.setPower(power);
        bl.setPower(power);
        br.setPower(-power);
    }
    public void Idle() {
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
    }
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Initializing...");
        telemetry.update();
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

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
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
        dontre = false;
        desYaw = false;
        checkYaw = false;
        desX = false;
        checkYaw2 = false;
        desZ = false;
        parkNeeded = true;
        step = 1;
        telemetry.addLine("Autonomous_RED_BACK Initialized...");
        telemetry.update();
        telemetry.addLine("Everything Works");
        telemetry.addLine("Ready to Start Autonomous");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            if (step == 1) {
                try{
	    /*
	    1. Move forward to the second bar
	   2. Move sideways through
	   the bar to the backdrop for 5 points
	   */
                    moveForward(0.5);
                    TimeUnit.MILLISECONDS.sleep(700);
                    Idle();
                    TimeUnit.MILLISECONDS.sleep(700);
                    telemetry.addLine("Turned right");
                    telemetry.update();
                    turnRight(0.5);
                    TimeUnit.MILLISECONDS.sleep(950);
                    moveForward(0.5);
                    TimeUnit.SECONDS.sleep(2);
                    Idle();
                    telemetry.addLine("Autonomous Setup Complete! :)");
                    telemetry.update();
                } catch(InterruptedException e){
//                    TODO: handle exception
                }
                step++;
            }
            telemetry.addLine("Moving to the April Tag...");
            telemetry.update();
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();
            if (currentDetections.size() != 0 && opModeIsActive())
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == RIGHT)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    double deadzone = 0.1;

                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                    Orientation rot = Orientation.getOrientation(tagOfInterest.pose.R, AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES);
                    do {
                        if (step == 2 &&!desYaw && opModeIsActive()) {
                            telemetry.addData("step: ", step);
                            telemetry.update();
                            if (rot.firstAngle < -5) {
                                turnLeft(0.2);
                            } else if (rot.firstAngle > 5) {
                                turnRight(0.2);
                            } else {
                                Idle();
                                desYaw = true;
                                telemetry.addData("desYaw: ", desYaw);
                                telemetry.update();
                                step++;
                            }
                        }
                        if (step == 3 && !desX && opModeIsActive()) {
                            telemetry.addData("step: ", step);
                            telemetry.update();
                            if(tagOfInterest.pose.x < -deadzone) {
                                strafeLeft(0.4);
                            }
                            else if(tagOfInterest.pose.x > deadzone) {
                                // do something else
                                strafeRight(0.4);
                            } else {
                                Idle();
                                desX = true;
                                telemetry.addData("desX: ", desX);
                                telemetry.update();
                                step++;
                            }
                        }
                        if (step == 4 && !checkYaw && opModeIsActive()) {
                            telemetry.addData("step: ", step);
                            telemetry.update();
                            if (rot.firstAngle < -5) {
                                turnLeft(0.25);
                            } else if (rot.firstAngle > 5) {
                                turnRight(0.25);
                            } else {
                                Idle();
                                checkYaw = true;
                                telemetry.addData("checkYaw: ", checkYaw);
                                telemetry.update();
                                step++;
                            }
                        }
                        if (step == 5 && !desZ && opModeIsActive()) {
                            telemetry.addData("step: ", step);
                            telemetry.update();
                            if (tagOfInterest.pose.z > 0.60) {
                                moveForward(0.20 );

                            } else {
                                Idle();
                                desZ = true;
                                telemetry.addData("desZ: ", desZ);
                                telemetry.update();
                                step++;
                            }
                        }
                        if (step == 6 && !checkYaw2 && opModeIsActive()) {
                            telemetry.addData("step: ", step);
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
                                step++;
                            }
                        }
                        if (desX && desZ) {
                            telemetry.addLine("Reached the april tag.");
                            telemetry.addLine("Placing the pixel");
                            telemetry.update();
                            serIn4.setPosition(0.75);
                            serIn5.setPosition(0.75);
                            TimeUnit.MILLISECONDS.sleep(500);
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
                            step++;
                            moveBackward(0.3);
                            TimeUnit.MILLISECONDS.sleep(300);
                            Idle();
                            TimeUnit.MILLISECONDS.sleep(300);
                            turnRight(0.5);
                            TimeUnit.MILLISECONDS.sleep(825);
                            Idle();
                            moveForward(0.5);
                            /*Later, this distance needs to be altered
                            because the distance between the april tags
                            makes the distance to park longer or shorter.
                             */
                            TimeUnit.MILLISECONDS.sleep(650);
                            Idle();
                            turnLeft(0.5);
                            TimeUnit.MILLISECONDS.sleep(700);
                            Idle();
                            moveForward(0.5);
                            TimeUnit.MILLISECONDS.sleep(500);
                            Idle();
                            telemetry.addLine("-------------------------------");
                            telemetry.addLine("Autonomous complete.");
                            telemetry.addLine("-------------------------------");
                            telemetry.update();
                        }

                    } while (step == 9 && desX && checkYaw && desZ && checkYaw2 && opModeIsActive() && parkNeeded);
                    /*
                        if (step == 66) {
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
                            step++;
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
    }

    void tagToTelemetry(AprilTagDetection detection)
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