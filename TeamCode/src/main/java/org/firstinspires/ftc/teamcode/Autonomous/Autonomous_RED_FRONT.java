package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.ArrayList;
import java.util.concurrent.TimeUnit;

@Autonomous(name = "Autonomous_RED_FRONT", group = "Autonomous", preselectTeleOp = "Payload_TeleOp")
public class Autonomous_RED_FRONT extends LinearOpMode
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
    int LEFT = 4;
    int MIDDLE = 5;
    int RIGHT = 6;
    AprilTagDetection tagOfInterest = null;
    boolean dontre;
    boolean desYaw;
    boolean checkYaw;
    boolean desX;
    boolean checkYaw2;
    boolean desZ;
    boolean checkYaw3;
    private DcMotor fr, fl, bl, br;

    public void moveForward(double power) {
        fl.setPower(-power);
        fr.setPower(-power);
        bl.setPower(-power);
        br.setPower(-power);
    }
    public void moveBackward(double power) {
        fl.setPower(-power);
        fr.setPower(-power);
        bl.setPower(-power);
        br.setPower(-power);
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
    public void Idle(double power) {
        fl.setPower(power);
        fr.setPower(power);
        bl.setPower(power);
        br.setPower(power);
    }
    @Override
    public void runOpMode()
    {
        telemetry.addLine("Initializing...");
        telemetry.update();
        fl = hardwareMap.get(DcMotor.class, "fl");
        br = hardwareMap.get(DcMotor.class, "br");
        fr = hardwareMap.get(DcMotor.class, "fr");
        bl = hardwareMap.get(DcMotor.class, "bl");

        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        fl.setDirection(DcMotorSimple.Direction.REVERSE);

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
        telemetry.addLine("Autonomous_RED_FRONT Initialized...");
        telemetry.update();
        telemetry.addLine("Everything Works! :)");
        telemetry.addLine("Ready to Start Autonomous");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            if (dontre == false) {
                try{
	    /*
	    1. Move forward to the second bar
	   2. Move sideways through
	   the bar to the backdrop for 5 points
	   */
                    moveForward(0.5);
                    TimeUnit.MILLISECONDS.sleep(785);
                    Idle(0);
                    TimeUnit.MILLISECONDS.sleep(700);
                    telemetry.addLine("Turned right");
                    telemetry.update();
                    turnRight(0.5);
                    TimeUnit.MILLISECONDS.sleep(710);
                    Idle(0);
                    telemetry.addLine("Autonomous Setup Complete! :)");
                    telemetry.update();
                } catch(InterruptedException e){
                    //TODO: handle exception
                }
                dontre = true;
            }
            telemetry.addLine("Moving to the AprilTag...");
            telemetry.update();
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();
            if (currentDetections.size() != 0 && opModeIsActive())
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    double deadzone = 0.2;

                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                    Orientation rot = Orientation.getOrientation(tagOfInterest.pose.R, AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES);
                    if (tagOfInterest.pose.z > 0.5) {
                        moveForward(1);
                    } else {
                        Idle(0);
                    }
                    if(tagOfInterest.pose.x < -deadzone) {
                        strafeLeft(1);
                    }
                    else if(tagOfInterest.pose.x > deadzone) {
                        // do something else
                        strafeRight(1);
                    } else {
                        Idle(0);
                    }
                    if (rot.firstAngle < -10) {
                        turnLeft(1);
                    } else if (rot.firstAngle > 10) {
                        turnRight(1);
                    } else {
                        Idle(0);
                    }
                }
                else
                {
                    Idle(0);
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
                Idle(0);
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