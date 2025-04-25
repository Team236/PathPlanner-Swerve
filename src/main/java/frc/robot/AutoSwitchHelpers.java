package frc.robot;


import edu.wpi.first.wpilibj.DigitalInput;

public class AutoSwitchHelpers
{
    private static DigitalInput autoSwitch1 = new DigitalInput(Constants.DIO_AUTO_1);
    private static DigitalInput autoSwitch2 = new DigitalInput(Constants.DIO_AUTO_2);
    private static DigitalInput autoSwitch3 = new DigitalInput(Constants.DIO_AUTO_3);
    private static DigitalInput autoSwitch4 = new DigitalInput(Constants.DIO_AUTO_4);
    

    //Changed to add the "NOT"  "!" since I think the swithes are inverted on the box (true is down)
    public boolean switchesAre(boolean a, boolean b, boolean c, boolean d)
    {
        if (autoSwitch1.get() == a && autoSwitch2.get() == b && autoSwitch3.get() == c && autoSwitch4.get() == d)
        {
            return true;
        }
        return false;
    }
}