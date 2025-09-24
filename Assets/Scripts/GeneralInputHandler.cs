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

    public virtual void OnLeft(Vector2 v)
    {
        _leftRaw = v;
        Debug.Log($"Generic Left: {_leftRaw}");
    }
    
    public virtual void OnLeft(InputValue v) => 
        OnLeft(v.Get<Vector2>());
    public virtual void OnLeft(InputAction.CallbackContext ctx) => 
        OnLeft(ctx.ReadValue<Vector2>());

    public virtual void OnRight(Vector2 v)
    {
        _rightRaw = v;
        Debug.Log($"Generic Right: {_rightRaw}");
    }
    public virtual void OnRight(InputValue v) =>
        OnRight(v.Get<Vector2>());
    public virtual void OnRight(InputAction.CallbackContext ctx) =>
        OnRight(ctx.ReadValue<Vector2>());

    public virtual void OnAux(InputValue v)
    {
        try
        {
            OnAux(v.Get<float>());
        }
        catch (InvalidOperationException)
        {
            try
            {
                OnAux(v.Get<Vector2>());
            }
            catch (InvalidOperationException)
            {
                throw new InvalidDataException(
                    "Aux input came from neither controllers or mouse."
                );
            }
        }
    }
    public virtual void OnAux(InputAction.CallbackContext ctx)
    {
        try
        {
            OnAux(ctx.ReadValue<float>());
        }
        catch (InvalidOperationException)
        {
            try
            {
                OnAux(ctx.ReadValue<Vector2>());
            }
            catch (InvalidOperationException)
            {
                throw new InvalidDataException(
                    "Aux input came from neither controllers or mouse."
                );
            }
        }
    }

    public virtual void OnAux(float v)
    {
        // from generic controller trigger or rc controller knob
        _aux = Mathf.Clamp(-1 + 2f * v, -1f, 1f);
        Debug.Log($"Generic Aux: {_aux}");
    }

    public virtual void OnAux(Vector2 v)
    {
        // mouse scroll wheel
        _aux = Mathf.Clamp(-1 + 2f * v.y, -1f, 1f);
        Debug.Log($"Generic Aux: {_aux}");
    }

    public virtual void OnTrigger(float v)
    {
        _trigger = v;
        Debug.Log($"Generic Trigger: {_trigger}");
    }
    public virtual void OnTrigger(InputValue v) => 
        OnTrigger(v.Get<float>());
    public virtual void OnTrigger(InputAction.CallbackContext ctx) => 
        OnTrigger(ctx.ReadValue<float>());
}