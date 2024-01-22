package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
    public class Colorsensor extends LinearOpMode {
    // Define a variable for our color sensor
    ColorSensor color;

    @Override
    public void runOpMode() {
        // Get the color sensor from hardwareMap
        color = hardwareMap.get(ColorSensor.class, "Color");

        // Wait for the Play button to be pressed
        waitForStart();

        // While the Op Mode is running, update the telemetry values.
        while (opModeIsActive()) {
            //nothing R27 G24 B16
            //green R136 G251 B108 to R328 G644 B277
            //white R1907 G2209 B1852
            //yellow R786 G620 B284 to R1783 G1499 B662
            //purple R1122 G1286 B1520
            String col = "";
            double RtoG = color.red() / color.green();
            double GtoB = color.green() / color.blue();
            double BtoR = color.blue() / color.red();


            telemetry.addData("Red", color.red());
            telemetry.addData("Green", color.green());
            telemetry.addData("Blue", color.blue());
            telemetry.addData("RtoG", RtoG);
            telemetry.addData("GtoB", GtoB);
            telemetry.addData("BtoR", BtoR);

            telemetry.update();
        }
    }
}