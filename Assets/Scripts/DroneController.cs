using System.Diagnostics.Tracing;
using UnityEngine;

public class DroneController : MonoBehaviour
{
    internal FsSm600Handler Handler;
    
    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Awake()
    {
        Handler = new();
        Debug.Log(Handler.Controls[0]);
    }

    // Update is called once per frame
    void Update()
    {
        Handler.Poll();
        // Debug.Log($"{Handler.Axes.Left}; {Handler.Axes.Right}; {Handler.Axes.Aux}; {Handler.Axes.Trigger};");
    }
}
