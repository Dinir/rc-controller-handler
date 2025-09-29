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
        private readonly SextupleAxesManager _deadzone;
        private readonly SextupleAxesManager _maxValue;
        private const float Epsilon = .000001f;
        private readonly bool[] _changed = new bool[6];

        public Vector2 Left
        {
            get => _left;
            set
            {
                value = new Vector2(
                    value.x >= 0 ? 
                        value.x >= _deadzone[0] ? value.x : 0 : 
                        value.x <= -_deadzone[0] ? value.x : 0,
                    value.y >= 0 ? 
                        value.y >= _deadzone[1] ? value.y : 0 :
                        value.y <= -_deadzone[1] ? value.y : 0
                );
                _left = new Vector2(
                    Mathf.Clamp(value.x / _maxValue[0], -_maxValue[0], _maxValue[0]),
                    Mathf.Clamp(value.y / _maxValue[1], -_maxValue[1], _maxValue[1])
                );
            }
        }
        public Vector2 Right
        {
            get => _right;
            set
            {
                value = new Vector2(
                    value.x >= 0 ? 
                        value.x >= _deadzone[2] ? value.x : 0 : 
                        value.x <= -_deadzone[2] ? value.x : 0,
                    value.y >= 0 ? 
                        value.y >= _deadzone[3] ? value.y : 0 :
                        value.y <= -_deadzone[3] ? value.y : 0
                );
                _right = new Vector2(
                    Mathf.Clamp(value.x / _maxValue[2], -_maxValue[2], _maxValue[2]),
                    Mathf.Clamp(value.y / _maxValue[3], -_maxValue[3], _maxValue[3])
                );
            }
        }
        public float Aux
        {
            get => _aux;
            set
            {
                value = value >= 0 ? 
                    value >= _deadzone[4] ? value : 0 : 
                    value <= -_deadzone[4] ? value : 0;
                _aux = Mathf.Clamp(value / _maxValue[4], -_maxValue[4], _maxValue[4]);
            }
        }
        public float Trigger
        {
            get => _trigger;
            set { _trigger = Mathf.Clamp(value / _maxValue[5], _deadzone[5], _maxValue[5]); }
        }

        public void SetDeadzoneLeft(Vector2 v) =>
            _deadzone._left = new Vector2(
                Mathf.Clamp01(Mathf.Abs(v.x)),
                Mathf.Clamp01(Mathf.Abs(v.y))
            );
        public void SetDeadzoneRight (Vector2 v) =>
            _deadzone._right = new Vector2(
                Mathf.Clamp01(Mathf.Abs(v.x)),
                Mathf.Clamp01(Mathf.Abs(v.y))
            );
        public void SetDeadzoneAux(float v) => _deadzone._aux = Mathf.Clamp01(Mathf.Abs(v));
        public void SetDeadzoneTrigger(float v) => _deadzone._trigger = Mathf.Clamp01(Mathf.Abs(v));
        
        public void SetMaxValueLeft(Vector2 v) =>
            _maxValue._left = new Vector2(
                Mathf.Clamp01(Mathf.Abs(v.x)),
                Mathf.Clamp01(Mathf.Abs(v.y))
            );
        public void SetMaxValueRight (Vector2 v) =>
            _maxValue._right = new Vector2(
                Mathf.Clamp01(Mathf.Abs(v.x)),
                Mathf.Clamp01(Mathf.Abs(v.y))
            );
        public void SetMaxValueAux(float v) => _maxValue._aux = Mathf.Clamp01(Mathf.Abs(v));
        public void SetMaxValueTrigger(float v) => _maxValue._trigger = Mathf.Clamp01(Mathf.Abs(v));

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
                    Mathf.Abs(this[i] - _deadzone[i]) > 0;
            }

            return _changed;
        }

        // ReSharper disable once MemberCanBePrivate.Global
        public SextupleAxesManager(Vector2? l, Vector2? r, float? a, float? t)
        {
            _deadzone = new SextupleAxesManager(new(0, 0), new(0, 0), 0, 0, true);
            _maxValue = new SextupleAxesManager(new(1f, 1f), new(1f, 1f), 1f, 1f, true);

            Left = l ?? Vector2.zero;
            Right = r ?? Vector2.zero;
            Aux = a ?? 0f;
            Trigger = t ?? 0f;
        }

        public SextupleAxesManager() : this(null, null, null, null)
        {
        }

        // for private instantiation to prevent recursive issues
        // ReSharper disable once GrammarMistakeInComment
        // ReSharper disable once UnusedParameter.Local
        private SextupleAxesManager(Vector2 l, Vector2 r, float a, float t, bool _)
        {
            _deadzone = null;
            _maxValue = null;
            _left = l;
            _right = r;
            _aux = a;
            _trigger = t;
        }
    }
}
