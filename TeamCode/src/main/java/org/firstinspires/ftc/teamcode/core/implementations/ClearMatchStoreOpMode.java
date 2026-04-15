package org.firstinspires.ftc.teamcode.core.implementations;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.core.OpModeCore;
import org.firstinspires.ftc.teamcode.utilities.MatchStateStore;

@TeleOp(name = "9 - Clear Match Store")
public class ClearMatchStoreOpMode extends OpModeCore {
    @Override
    protected void onRun() {
        MatchStateStore.clear();
        prettyTelem.info("Match state store cleared.");
        prettyTelem.addData("Status", ()-> "Cleared");
        prettyTelem.update();
    }
}
