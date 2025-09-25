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
        if ( v is not null && v.Get() is not null )
        {
            GeneralHandler.OnLeft(v);
        }
        else
        {
            Handler.OnLeft();
        }
    }

    public void OnRight(InputValue v)
    {
        if ( v is not null && v.Get() is not null )
        {
            GeneralHandler.OnRight(v);
        }
        else
        {
            Handler.OnRight();
        }
    }
    
    public void OnAux(InputValue v)
    {
        if ( v is not null && v.Get() is not null )
        {
            GeneralHandler.OnAux(v);
        }
        else
        { 
            Handler.OnAux();
        }
    }
    
    public void OnTrigger(InputValue v)
    {
        if ( v is not null && v.Get() is not null )
        {
            GeneralHandler.OnTrigger(v);
        }
        else
        {
            Handler.OnTrigger();
        }
    }
}
