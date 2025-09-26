using System;
using UnityEngine;

namespace ControlHandler
{
    /// <summary>
    /// Getter and setter for six-axis controls.
    /// The axes are arranged in an order that would comply with common RC controllers, hopefully:
    /// <list type="bullet">
    ///   <item>Left (Vector2)</item>
    ///   <item>Right (Vector2)</item>
    ///   <item>Aux (float)</item>
    ///   <item>Trigger (float)</item>
    /// </list>
    /// </summary>
    public class SextupleAxesManager
    {
        private Vector2 _left;
        private Vector2 _right;
        private float _aux;
        private float _trigger;
        public readonly SextupleAxesManager Deadzone;
        public const float Epsilon = .000001f;
        private bool[] _changed = new bool[6];

        public Vector2 Left
        {
            get => _left;
            set => _left = new Vector2(
                Mathf.Clamp(value.x / Deadzone[0], -Deadzone[0], Deadzone[0]),
                Mathf.Clamp(value.y / Deadzone[1], -Deadzone[1], Deadzone[1])
            );
        }

        public Vector2 Right
        {
            get => _right;
            set => _right = new Vector2(
                Mathf.Clamp(value.x / Deadzone[2], -Deadzone[2], Deadzone[2]),
                Mathf.Clamp(value.y / Deadzone[3], -Deadzone[3], Deadzone[3])
            );
        }

        public float Aux
        {
            get => _aux;
            set => _aux = Mathf.Clamp(value / Deadzone[4], -Deadzone[4], Deadzone[4]);
        }

        public float Trigger
        {
            get => _trigger;
            set => _trigger = Mathf.Clamp(value / Deadzone[5], 0f, Deadzone[5]);
        }
        
        public void SetDeadzoneLeft(Vector2 v) =>
            _left = new Vector2(
                Mathf.Clamp01(v.x),
                Mathf.Clamp01(v.y)
            );
        public void SetDeadzoneRight (Vector2 v) =>
            _right = new Vector2(
                Mathf.Clamp01(v.x),
                Mathf.Clamp01(v.y)
            );
        public void SetDeadzoneAux(float v) => _aux = Mathf.Clamp01(v);
        public void SetDeadzoneTrigger(float v) => _trigger = Mathf.Clamp01(v);

        /// <summary>
        /// Make the axis values available in the form of array elements.
        /// </summary>
        /// <param name="i">Index for the array form.</param>
        /// <exception cref="IndexOutOfRangeException"></exception>
        public float this[int i]
        {
            get => i switch
            {
                0 => _left.x,
                1 => _left.y,
                2 => _right.x,
                3 => _right.y,
                4 => _aux,
                5 => _trigger,
                _ => throw new IndexOutOfRangeException(nameof(i))
            };
            set
            {
                switch (i)
                {
                    case 0: Left = new Vector2(value, _left.y); break;
                    case 1: Left = new Vector2(_left.x, value); break;
                    case 2: Right = new Vector2(value, _right.y); break;
                    case 3: Right = new Vector2(_right.x, value); break;
                    case 4: Aux = value; break;
                    case 5: Trigger = value; break;
                    default: throw new IndexOutOfRangeException(nameof(i));
                }
            }
        }

        public bool[] GetChanges(SextupleAxesManager prev, float e = Epsilon)
        {
            if (prev == null)
                throw new ArgumentNullException(nameof(prev));

            for (int i = 0; i < 6; i++)
            {
                _changed[i] =
                    Mathf.Abs(this[i] - prev[i]) > e ||
                    this[i] >= Deadzone[i];
            }

            return _changed;
        }

        public SextupleAxesManager(Vector2? l, Vector2? r, float? a, float? t)
        {
            Deadzone = new SextupleAxesManager(new(1f, 1f), new(1f, 1f), 1f, 1f, true);

            Left = l ?? Vector2.zero;
            Right = r ?? Vector2.zero;
            Aux = a ?? 0f;
            Trigger = t ?? 0f;
        }

        public SextupleAxesManager() : this(null, null, null, null)
        {
        }

        // for private instantiation to prevent recursive issues
        private SextupleAxesManager(Vector2 l, Vector2 r, float a, float t, bool _)
        {
            Deadzone = null;
            _left = l;
            _right = r;
            _aux = a;
            _trigger = t;
        }
    }
}
