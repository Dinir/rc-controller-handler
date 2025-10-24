using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UnityEngine.InputSystem;
using UnityEngine.InputSystem.Controls;

namespace ControlHandler
{
    public class RCControllerHandler
    {
        internal static readonly int ControlAmount = 6;
        internal InputDevice Device;
        internal readonly InputControl[] Controls = new InputControl[6];
        internal readonly SextupleAxesManager Axes = new();
        internal readonly SextupleAxesManager PrevAxes = new();
        internal bool[] Changes = new bool[6];
        internal bool[] Activeness = new bool[6];

        public RCControllerHandler(IEnumerable<string> names, string[] controls)
        {
            if (names == null)
                throw new ArgumentException("No controller names provided.");
            if (controls.Length != ControlAmount)
                throw new ArgumentException($"Number of controls must be {ControlAmount}.");

            TryToConnect(names, controls);
        }

        public void TryToConnect(IEnumerable<string> names, string[] controls)
        {
            Device = InputSystem.devices.FirstOrDefault(d =>
            {
                if (string.IsNullOrEmpty(d.name)) return false;
                return names.Any(n => d.name.Contains(n, StringComparison.OrdinalIgnoreCase
                ));
            });

            if (Device == null)
            {
                Debug.LogWarning("RC controller not found.");
                return;
            }

            for (int i = 0; i < ControlAmount; i++)
            {
                Controls[i] = Device.TryGetChildControl(controls[i]);
            }
        }

        /// <summary returns="bool">
        /// For every frame, this does three things:
        /// <list type="bullet">
        /// <item>Store last updated axis values.</item>
        /// <item>Update axis values with current values.</item>
        /// <item>Update what axis has changed.</item>
        /// </list>
        /// </summary>
        public bool Poll()
        {
            try
            {
                for (int i = 0; i < ControlAmount; i++)
                {
                    PrevAxes[i] = Axes[i];
                    Axes[i] = ((AxisControl)Controls[i]).ReadValue();
                }

                Changes = Axes.GetChanges(PrevAxes);
                Activeness = Axes.GetActiveness();
                return true;
            }
            catch (Exception)
            {
                Device = null;
                Debug.LogWarning("Connection to RC controller is lost.");
                return false;
            }
        }

        public virtual Vector2 OnLeft() => Axes.Left;
        public virtual Vector2 OnRight() => Axes.Right;
        public virtual float OnAux() => AuxNormalize(Axes.Aux);
        public virtual float OnTrigger() => Axes.Trigger;

        public void SendMessages(GameObject receiver, bool[] stateChanges)
        {
            if (stateChanges[0] || stateChanges[1])
                receiver.SendMessage(
                    "OnLeft", 0, SendMessageOptions.DontRequireReceiver
                );
            if (stateChanges[2] || stateChanges[3])
                receiver.SendMessage(
                    "OnRight", 0, SendMessageOptions.DontRequireReceiver
                );
            if (stateChanges[4])
                receiver.SendMessage(
                    "OnAux", 0, SendMessageOptions.DontRequireReceiver
                );
            if (stateChanges[5])
                receiver.SendMessage(
                    "OnTrigger", 0, SendMessageOptions.DontRequireReceiver
                );
        }

        private static float AuxNormalize(float v) =>
            v >= 0 && v <= 1f ? v : Mathf.Clamp01((v + 1f) * .5f);
    }
}