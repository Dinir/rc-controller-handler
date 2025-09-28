using System;
using System.IO;
using UnityEngine;
using UnityEngine.InputSystem;

namespace ControlHandler
{
    public class GeneralInputHandler
    {
        private protected Vector2 LeftRaw;
        private protected Vector2 RightRaw;
        private protected float Aux;
        private protected float Trigger;

        public virtual Vector2 OnLeft(InputValue v)
        {
            LeftRaw = v.Get<Vector2>();
            // Debug.Log($"Generic Left: {LeftRaw}");

            return LeftRaw;
        }

        public virtual Vector2 OnRight(InputValue v)
        {
            RightRaw = v.Get<Vector2>();
            // Debug.Log($"Generic Right: {RightRaw}");

            return RightRaw;
        }

        public virtual float OnAux(InputValue v)
        {
            try
            {
                // from generic controller trigger or rc controller knob
                Aux = Mathf.Clamp(-1 + 2f * v.Get<float>(), -1f, 1f);
            }
            catch (InvalidOperationException)
            {
                try
                {
                    // mouse scroll wheel
                    Aux = Mathf.Clamp(Aux + .1f * v.Get<Vector2>().y, -1f, 1f);
                }
                catch (InvalidOperationException)
                {
                    throw new InvalidDataException(
                        "Aux input came from neither controllers or mouse."
                    );
                }
            }
            // Debug.Log($"Generic Aux: {Aux}");

            return Aux;
        }

        public virtual float OnTrigger(InputValue v)
        {
            Trigger = v.Get<float>();
            // Debug.Log($"Generic Trigger: {Trigger}");

            return Trigger;
        }
    }
}
