/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;


/**
 * This OpMode illustrates the basics of using the Vuforia engine to determine
 * the identity of Vuforia VuMarks encountered on the field. The code is structured as
 * a LinearOpMode. It shares much structure with {@ link ConceptVuforiaNavigation}; we do not here
 * duplicate the core Vuforia documentation found there, but rather instead focus on the
 * differences between the use of Vuforia for navigation vs VuMark identification.
 *
 * @see ConceptVuforiaNavigation
 * @see VuforiaLocalizer
 * @see VuforiaTrackableDefaultListener
 * see  ftc_app/doc/tutorial/FTC_FieldCoordinateSystemDefinition.pdf
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained in {@ link ConceptVuforiaNavigation}.
 */
@TeleOp(name="Mec: SBDRukusMarkHW", group="Mec")
public class SBDRukusMarkHW extends OpMode {



    private robotHWconnected autoConnectedHW = robotHWconnected.VuforWebcam;
    private HardwareRukusMecBot MecBot ;
    private boolean targetVisible = false;
    private static final float mmPerInch        = 25.4f;
    private static final float mmFTCFieldWidth  = (12*6) * mmPerInch;       // the width of the FTC field (from the center point to the outer panels)
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor



    @Override
    public void init () {
        MecBot = new HardwareRukusMecBot(autoConnectedHW) ;
        MecBot.init(hardwareMap, Color.Red);

        composeTelemetry();

    }

    @Override
    public void start() {

        MecBot.start();

        telemetry.addData(">", "Press Play to start");
        telemetry.update();

    }

    @Override
    public void loop () {
        MecBot.VuReader.UpdateLocation();
        //        RelicRecoveryVuMark vuMark;
        targetVisible = MecBot.VuReader.isTargetVisible();
//        OmniBot.VuReader.updateVuforiaCoords();

        RobotLog.i("X value is " + MecBot.VuReader.getVuforiaCoords(HardwareRukusVuforia.vuForiaCoord.tZ));
        RobotLog.i("X value Rot is " + MecBot.VuReader.getVuforiaCoords(HardwareRukusVuforia.vuForiaCoord.rZ));
        if(targetVisible) {
            telemetry.addData("Visible Target", MecBot.VuReader.getTrackableName());
            VectorF translation = MecBot.VuReader.getTranslation();
            telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                    translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);
            Orientation rotation = MecBot.VuReader.getRotation();
            telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
        } else {
            telemetry.addData("Visible Target", "none");
        }
        telemetry.update();
    }

    void composeTelemetry() {

        MecBot.addTelemetry(telemetry);
        //OmniBot.getTelemetry(telemetry);
    }
}
