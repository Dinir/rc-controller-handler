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

    // event handling for generic controllers (gamepads and keyboards + mouses)
    public void OnLeft(InputValue v)
    {
            ActionLeft(GeneralHandler.OnLeft(v));
    }
    public void OnRight(InputValue v)
    {
            ActionRight(GeneralHandler.OnRight(v));
    }
    public void OnAux(InputValue v)
    {
            ActionAux(GeneralHandler.OnAux(v));
    }
    public void OnTrigger(InputValue v)
    {
            ActionTrigger(GeneralHandler.OnTrigger(v));
    }

    // event handling for RC Controllers
    public void OnLeft(int _)
    {
        ActionLeft(Handler.OnLeft());    
    }
    public void OnRight(int _)
    {
        ActionRight(Handler.OnRight());
    }
    public void OnAux(int _)
    {
        ActionAux(Handler.OnAux());
    }
    public void OnTrigger(int _)
    {
        ActionTrigger(Handler.OnTrigger());
    }

    // action for the attached game object
    public void ActionLeft(Vector2 v)
    {
    }
    public void ActionRight(Vector2 v)
    {
    }
    public void ActionAux(float v)
    {
    }
    public void ActionTrigger(float v)
    {
    }
}
