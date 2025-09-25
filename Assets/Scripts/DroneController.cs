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
        HandlerFound = Handler.Device != null;
        PrevAxes = new();
        GeneralAxes = new();
    }

    void Update()
    {
        // for RC Controller
        if (HandlerFound)
        {
            Handler.Poll();
            Handler.SendMessages(gameObject, Handler.Changes);
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
                HandlerFound = Handler.Device != null;
            }
        }
    }

    // event handling for RC Controller
    public void OnLeft(InputValue v)
    {
            GeneralHandler.OnLeft(v);
    }

    public void OnRight(InputValue v)
    {
            GeneralHandler.OnRight(v);
    }
    
    public void OnAux(InputValue v)
    {
            GeneralHandler.OnAux(v);
    }
    
    public void OnTrigger(InputValue v)
    {
            GeneralHandler.OnTrigger(v);
    }

    public void OnLeft(int _)
    {
        Handler.OnLeft();    
    }

    public void OnRight(int _)
    {
        Handler.OnRight();
    }

    public void OnAux(int _)
    {
        Handler.OnAux();
    }

    public void OnTrigger(int _)
    {
        Handler.OnTrigger();
    }
}
