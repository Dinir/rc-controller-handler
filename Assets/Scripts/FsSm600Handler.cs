using System.Collections.Generic;

public class FsSm600Handler : RCControllerHandler
{
    private static readonly string[] Names = 
    {
        "RC Simulator - AeroFly Controller",
        "PengFei Model RC Simulator - AeroFly Controller"
    };
    private static readonly string[] ControlNames =
    {
        "stick/x", "slider", "rz", "stick/y", "z", "trigger"
    };
    public FsSm600Handler() : base(Names, ControlNames) {}

    public void TryToConnect()
    {
        TryToConnect(Names, ControlNames);
    }
}