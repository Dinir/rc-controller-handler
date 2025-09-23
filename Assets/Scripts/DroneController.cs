using System.Diagnostics.Tracing;
using UnityEngine;

public class DroneController : MonoBehaviour
{
    internal FsSm600Handler Handler;
    internal SextupleAxesManager PrevAxes;
    internal bool[] Changed = new bool[6];
    
    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Awake()
    {
        Handler = new();
        PrevAxes = new();
    }

    // Update is called once per frame
    void Update()
    {
        Handler.Poll(); 
        Handler.SendMessages(gameObject, Handler.Changes);
    }

    public void OnLeft()
    {
        Debug.Log($"Left: {Handler.Axes.Left}");
    }

    public void OnRight()
    {
        Debug.Log($"Right: {Handler.Axes.Right}");
    }
    
    public void OnAux()
    {
        Debug.Log($"Aux: {Handler.Axes.Aux}");
    }
    
    public void OnTrigger()
    {
        Debug.Log($"Trigger: {Handler.Axes.Trigger}");
    }
}
