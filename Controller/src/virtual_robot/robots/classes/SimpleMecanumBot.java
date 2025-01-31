package virtual_robot.robots.classes;

import virtual_robot.controller.BotConfig;
import virtual_robot.util.AngleUtils;


@BotConfig(name = "SimpleMecanum Bot", filename = "simple_mecanum_bot")
public class SimpleMecanumBot extends MecanumPhysicsBase {

    public SimpleMecanumBot(){
        super();
    }

    public void initialize(){
        super.initialize();
        hardwareMap.setActive(true);
        hardwareMap.setActive(false);
    }

    protected void createHardwareMap(){
        super.createHardwareMap();
    }

    public synchronized void updateStateAndSensors(double millis){
        super.updateStateAndSensors(millis);
    }

    public synchronized void updateDisplay(){
        super.updateDisplay();
    }

    public void powerDownAndReset(){
        super.powerDownAndReset();
    }
}
