package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import java.util.concurrent.TimeUnit;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp

public class Payload_TeleOp extends LinearOpMode {
    private DcMotor fr, fl, br, bl, dcAc1, dcAc2;
    private Servo serAc1, serAc2, serIn5, serAir;
    private CRServo serIn1, serIn2;

    public void runOpMode() {
        fl = hardwareMap.get(DcMotor.class, "fl");
        br = hardwareMap.get(DcMotor.class, "br");
        fr = hardwareMap.get(DcMotor.class, "fr");
        bl = hardwareMap.get(DcMotor.class, "bl");
        dcAc1 = hardwareMap.get(DcMotor.class, "dcAc1");
        serAir = hardwareMap.get(Servo.class, "serAir");
        dcAc2 = hardwareMap.get(DcMotor.class, "dcAc2");
        serAc1 = hardwareMap.get(Servo.class, "serAc1");
        serAc2 = hardwareMap.get(Servo.class, "serAc2");
        serIn1 = hardwareMap.get(CRServo.class, "serIn1");
        serIn2 = hardwareMap.get(CRServo.class, "serIn2");
        serIn5 = hardwareMap.get(Servo.class, "serIn5");
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        serAc2.setDirection(Servo.Direction.REVERSE);
        serAir.setDirection(Servo.Direction.REVERSE);
        serIn5.setDirection(Servo.Direction.FORWARD);
        serAc1.setPosition(0.97);
        serAc2.setPosition(0.97);
        serAir.setPosition(0);
        telemetry.addLine("Motors Assigned and Attached");
        telemetry.addLine("SerAc1 Pos: " + serAc1.getPosition());
        telemetry.addLine("SerAc2 Pos: " + serAc2.getPosition());
        telemetry.addLine("SerIn5: " + String.valueOf(serIn5.getPosition()));
        telemetry.addLine("SerIn5 Updated: " + String.valueOf(serIn5.getPosition()));
        telemetry.addLine("BLAST OFF!!!");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            double lx = gamepad1.left_stick_x;
            double ly = gamepad1.left_stick_y;
            double rx = gamepad1.right_stick_x;
            double rt = gamepad2.right_trigger;
            double lt = gamepad2.left_trigger;
            double ly2 = gamepad2.left_stick_y;
            double ry2 = gamepad2.right_stick_y;
            if (gamepad1.right_bumper == false) {
                fl.setPower(ly + lx - rx);
                fr.setPower(ly - lx + rx);
                bl.setPower(ly - lx - rx);
                br.setPower(ly + lx + rx);
            } else {
                if (gamepad1.right_bumper == true) {
                    fl.setPower((ly + lx - rx) / 2);
                    fr.setPower((ly - lx + rx) / 2);
                    bl.setPower((ly - lx - rx) / 2);
                    br.setPower((ly + lx + rx) / 2);
                }

            }
            dcAc1.setPower(-ly2);
            dcAc2.setPower(-ry2);
            serIn1.setPower(rt);
            serIn2.setPower(-rt);
            serIn1.setPower(-lt);
            serIn2.setPower(lt);
            if (gamepad2.right_stick_y > 0 || gamepad2.right_stick_y < 0){
                telemetry.addLine("right stick" + gamepad2.right_stick_y);
                telemetry.update();
            }
            if (gamepad2.dpad_up == true || gamepad1.dpad_up == true) {
                dcAc1.setPower(1.5);
                dcAc2.setPower(1.5);
                if (gamepad2.dpad_up == false || gamepad1.dpad_up == false) {
                    dcAc1.setPower(0);
                    dcAc2.setPower(0);
                }
            }
            if (gamepad2.dpad_down == true) {
                dcAc1.setPower(-1);
                dcAc2.setPower(-1);
                if (gamepad2.dpad_down == false || gamepad1.dpad_down == false) {
                    dcAc1.setPower(0);
                    dcAc2.setPower(0);
                    }
            }
            if (gamepad2.y == true) {
                serAc1.setPosition(0.65);
                serAc2.setPosition(0.65);
                telemetry.addLine("serAc2 " + serAc2.getPosition());
                telemetry.update();
            }
            if (gamepad2.a == true) {
                serAc1.setPosition(0.78);
                serAc2.setPosition(0.78);
                try {
                    telemetry.addLine("First: serAc2 " + serAc2.getPosition());
                    telemetry.update();
                    TimeUnit.SECONDS.sleep(1);
                    serAc1.setPosition(0.95);
                    serAc2.setPosition(0.95);
                    telemetry.addLine("Second: serAc2 " + serAc2.getPosition());
                    telemetry.update();
                    TimeUnit.SECONDS.sleep(1);
                } catch (InterruptedException e){
                    //TODO: handle exception
                }
                telemetry.addLine("Final: serAc2 " + serAc2.getPosition());
                telemetry.update();
                serAc1.setPosition(0.97);
                serAc2.setPosition(0.97);
            }
            if (gamepad2.right_bumper == true) {
                serIn5.setPosition(0);
                telemetry.addLine("serIn5 " + serIn5.getPosition());
                telemetry.update();
            }
            if (gamepad2.left_bumper == true) {
                serIn5.setPosition(0.34);
                telemetry.addLine("serIn5 " + serIn5.getPosition());
                telemetry.update();
            }
            if (gamepad2.start == true) {
                serIn5.setPosition(0.13);
                telemetry.addLine("serIn5 " + serIn5.getPosition());
                telemetry.update();
            }
            if (gamepad2.x == true) {
                serAir.setPosition(60);
            }
            if (gamepad2.b == true) {
                serAir.setPosition(0);
            }
            if (gamepad2.back == true) {
                serAc1.setPosition(0.78);
                serAc2.setPosition(0.78);
            }
        }
    }
}
