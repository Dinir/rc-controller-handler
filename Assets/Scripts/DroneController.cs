using System.Diagnostics.Tracing;
using JetBrains.Annotations;
using UnityEngine;
using UnityEngine.InputSystem;

public class DroneController : MonoBehaviour
{
    internal FsSm600Handler Handler;
    internal GeneralInputHandler GeneralHandler;
    internal bool HandlerFound;
    internal float HandlerPollTime;
    internal readonly float HandlerPollTimer = 3f;
    internal SextupleAxesManager PrevAxes;
    internal SextupleAxesManager GeneralAxes;
    internal bool[] Changed = new bool[6];
    
    void Awake()
    {
        Handler = new();
        GeneralHandler = new();
        PrevAxes = new();
        GeneralAxes = new();
    }

    void Update()
    {
        HandlerFound = Handler.Device != null;
        // for RC Controller
        if (HandlerFound)
        {
            Handler.Poll();
            Handler.SendMessages(this, Handler.Changes);
        }
        else
        {
            HandlerPollTime += Time.deltaTime;
            if (HandlerPollTime >= HandlerPollTimer)
            {
                HandlerPollTime = 0;
                Handler.TryToConnect(
                    FsSm600Handler.Names, FsSm600Handler.ControlNames
                );
            }
        }
    }

    // RC controller (zero-arg)
    public void OnLeft()    { Handler.OnLeft(); }
    public void OnRight()   { Handler.OnRight(); }
    public void OnAux()     { Handler.OnAux(); }
    public void OnTrigger() { Handler.OnTrigger(); }

    // Generic controllers — PlayerInput "Send Messages"
    public void OnLeft(InputValue v)    { GeneralHandler.OnLeft(v); }
    public void OnRight(InputValue v)   { GeneralHandler.OnRight(v); }
    public void OnAux(InputValue v)     { GeneralHandler.OnAux(v); }
    public void OnTrigger(InputValue v) { GeneralHandler.OnTrigger(v); }

    // Generic controllers — PlayerInput "Invoke C# Events"
    public void OnLeft(InputAction.CallbackContext ctx)    { GeneralHandler.OnLeft(ctx); }
    public void OnRight(InputAction.CallbackContext ctx)   { GeneralHandler.OnRight(ctx); }
    public void OnAux(InputAction.CallbackContext ctx)     { GeneralHandler.OnAux(ctx); }
    public void OnTrigger(InputAction.CallbackContext ctx) { GeneralHandler.OnTrigger(ctx); }
}
