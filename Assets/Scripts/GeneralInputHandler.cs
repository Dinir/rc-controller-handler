using UnityEngine;
using UnityEngine.InputSystem;

public class GeneralInputHandler
{
    private Vector2 _leftRaw;
    private Vector2 _rightRaw;
    private float _aux;
    private float _trigger;

    void OnLeft(InputValue v)
    {
        _leftRaw = v.Get<Vector2>();
    }

    void OnRight(InputValue v)
    {
        _rightRaw = v.Get<Vector2>();
    }

    void OnAux(InputValue v)
    {
        _aux = v.Get<float>();
    }

    void OnTrigger(InputValue v)
    {
        _trigger = v.Get<float>();
    }
}