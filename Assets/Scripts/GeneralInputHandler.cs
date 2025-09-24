using System;
using System.IO;
using UnityEngine;
using UnityEngine.InputSystem;

public class GeneralInputHandler
{
    private Vector2 _leftRaw;
    private Vector2 _rightRaw;
    private float _aux;
    private float _trigger;

    public virtual void OnLeft(InputValue v)
    {
        _leftRaw = v.Get<Vector2>();
        Debug.Log($"Generic Left: {_leftRaw}");
    }

    public virtual void OnRight(InputValue v)
    {
        _rightRaw = v.Get<Vector2>();
        Debug.Log($"Generic Right: {_rightRaw}");
    }

    public virtual void OnAux(InputValue v)
    {
        try
        {
            // from generic controller trigger or rc controller knob
            _aux = Mathf.Clamp(-1 + 2f * v.Get<float>(), -1f, 1f);
        }
        catch (InvalidOperationException)
        {
            try
            {
                // mouse scroll wheel
                _aux = Mathf.Clamp(_aux + .1f * v.Get<Vector2>().y, -1f, 1f);
            }
            catch (InvalidOperationException)
            {
                throw new InvalidDataException(
                    "Aux input came from neither controllers or mouse."
                );
            }
        }
        
        Debug.Log($"Generic Aux: {_aux}");
    }

    public virtual void OnTrigger(InputValue v)
    {
        _trigger = v.Get<float>();
        Debug.Log($"Generic Trigger: {_trigger}");
    }
}