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
        private protected float AuxLock;
        private protected float Trigger;

        public virtual Vector2 OnLeft(InputValue v)
        {
            LeftRaw = v.Get<Vector2>();

            return LeftRaw;
        }
        public virtual Vector2 OnRight(InputValue v)
        {
            RightRaw = v.Get<Vector2>();

            return RightRaw;
        }
        public virtual float OnAux(InputValue v)
        {
            try
            {
                Aux = v.Get<float>();
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

            return Aux;
        }
        public virtual float OnAuxLock(InputValue v)
        {
            // rc controller aux can hold a value,
            // while gamepad triggers can't.
            // the bumper button will be used as a switch
            // that enables lerping into trigger value.
            AuxLock = v.Get<float>();
            
            return AuxLock;
        }
        public virtual float OnTrigger(InputValue v)
        {
            Trigger = v.Get<float>();

            return Trigger;
        }
    }
}
