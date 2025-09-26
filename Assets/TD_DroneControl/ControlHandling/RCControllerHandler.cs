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
                return true;
            }
            catch (Exception e)
            {
                Device = null;
                Debug.LogWarning("Connection to RC controller is lost.");
                return false;
            }
        }

        public virtual Vector2 OnLeft()
        {
            // Debug.Log($"RC Left: {Axes.Left}");

            return Axes.Left;
        }

        public virtual Vector2 OnRight()
        {
            // Debug.Log($"RC Right: {Axes.Right}");

            return Axes.Right;
        }

        public virtual float OnAux()
        {
            // Debug.Log($"RC Aux: {Axes.Aux}");

            return Axes.Aux;
        }

        public virtual float OnTrigger()
        {
            // Debug.Log($"RC Trigger: {Axes.Trigger}");

            return Axes.Trigger;
        }

        public void SendMessages(GameObject receiver, bool[] changes)
        {
            if (changes[0] || changes[1])
                receiver.SendMessage(
                    "OnLeft", 0, SendMessageOptions.DontRequireReceiver
                );
            if (changes[2] || changes[3])
                receiver.SendMessage(
                    "OnRight", 0, SendMessageOptions.DontRequireReceiver
                );
            if (changes[4])
                receiver.SendMessage(
                    "OnAux", 0, SendMessageOptions.DontRequireReceiver
                );
            if (changes[5])
                receiver.SendMessage(
                    "OnTrigger", 0, SendMessageOptions.DontRequireReceiver
                );
        }
    }
}