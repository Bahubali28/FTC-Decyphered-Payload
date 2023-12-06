package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;
import java.util.concurrent.TimeUnit;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp

public class Autonomous_RED_BACK extends LinearOpMode{
    private DcMotor fr, fl, br, bl; // Create the motors' variables
    public void runOpMode() {
        int dis = 1;
        fl = hardwareMap.get(DcMotor.class, "fl"); // Define all of the motors
        br = hardwareMap.get(DcMotor.class, "br");
        fr = hardwareMap.get(DcMotor.class, "fr");
        bl = hardwareMap.get(DcMotor.class, "bl");
        bl.setDirection(DcMotorSimple.Direction.REVERSE);// Reverse the direction of the motors to use mechanum wheels.
        fl.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addLine("Motors are connected for Autonomous mode."); // Telemetry output
        telemetry.addLine("Mode: Red Back");
        telemetry.addLine("Yay! Everything works!");
        telemetry.addLine("-----------------------------");
        telemetry.update(); // Print them all in telemetry
        waitForStart();

        if (opModeIsActive()){ // Actually start the code
            try{
	/* 1. Move forward by 2 inches
	2. Move sideways through
	the bar to the backdrop */

                fl.setPower(-dis); // Motors are in reverse, so the value has to be negative to move forward
                fr.setPower(dis);
                bl.setPower(-dis);
                br.setPower(dis);
                TimeUnit.MILLISECONDS.sleep(500);
                fl.setPower(dis); // Motors are in reverse, so the value has to be posative to move sideways
                fr.setPower(dis);
                bl.setPower(dis);
                br.setPower(dis);
                TimeUnit.SECONDS.sleep(5);
            } catch(InterruptedException e){
                //TODO: handle exception
            }
        }
    }
}