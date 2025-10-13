using System;
using System.Collections.Generic;
using ControlHandler;
using UnityEngine;
using UnityEngine.InputSystem;
// ReSharper disable PossibleLossOfFraction

namespace DroneMovement
{
    public enum WingDir { CCW = -1, CW = 1 }
    public enum WingPos { Front = 1, Middle = 0, Back = -1 }
     
    [RequireComponent(typeof(Rigidbody), typeof(PlayerInput))]
    public class DroneController : MonoBehaviour
    {
        private const int InspectorSpace = 14;
        private FsSm600Handler _handler;
        private GeneralInputHandler _generalHandler;
        private bool _handlerFound;
        private float _handlerPollTime;
        private readonly float _handlerPollTimer = 3f;
        private SextupleAxesManager _prevAxes;
        private SextupleAxesManager _generalAxes;
        private bool _triggerToggle; // from OnTrigger
        private float _knobMix; // from OnAux
        
        private Vector2 _inputLeft;
        private Vector2 _inputRight;
        
        private readonly Quaternion[] _wingRotations = new Quaternion[6];
        private readonly float[] _wingsXzDistances = new float[6];
        private readonly Quaternion[] _wingRotationsFromStrafe = new Quaternion[6];

        [Header("Configuration")]
        [SerializeField] private bool isPowered;
        [SerializeField] private bool autoHover = true;
        [Tooltip("N (kg·m/s²), calculates automatically when Auto Hover is on.")]
        [SerializeField] [Min(0)] private float baseThrottleForce = .68125f;
        [SerializeField] [Min(0)] private float throttleForce = 10f;
        
        [Header("Rotors")]
        [Tooltip("N")]
        [SerializeField] [Min(0)] private float maxThrustPerRotor = 12f;
        [Tooltip("torque/N, reaction per thrust")]
        [SerializeField] [Min(0)] private float yawTorquePerThrust = .075f;
        [Space(InspectorSpace/2)]
        [Tooltip("s^-1, how fast rotor follows commands")]
        [SerializeField] private float commandSensitivity = 12f;
        [Space(InspectorSpace/2)]
        [Tooltip("per-rotor delta from yaw command")]
        [SerializeField] private float yawMix = .30f;
        [Tooltip("per-rotor delta from roll command")]
        [SerializeField] private float rollMix = .45f;
        [Tooltip("per-rotor delta from pitch command")]
        [SerializeField] private float pitchMix = .45f;
        [Space(InspectorSpace/2)]
        [Tooltip("scales torque command")]
        [SerializeField] [Min(0)] private float torqueMultiplier = 1f;
        [Tooltip("scales lift command")]
        [SerializeField] [Min(0)] private float liftMultiplier = .5f;

        private float[] _rotorCmd;
        private float[] _rotorTarget;
        private float[] _rotorTransition;
        
        private Vector3 _worldUp;
        private Vector3 _currentUp;
        
        [Header("Physical Property")]
        [SerializeField] [Min(0)] private float strafeRate = 1f;
        [Tooltip("An invisible plane sitting on the wings attitude in drone game object, tilts as xz movement input is received.")]
        [SerializeField] HelperPlane xzPlane;
        [Tooltip("deg, how much the helper plane can tilt.")]
        [SerializeField] [Min(0)] private float maxTiltDegree = 30f;
        [Tooltip("deg")]
        [SerializeField] [Range(0, 1f)] private float tiltCompensationMinCos = .35f;
        [Space(InspectorSpace)]
        [Tooltip("deg/sec/N")]
        [SerializeField] [Min(0)] private float wingRotationMultiplier = 16f; // for cosmetic wing rotation
        private float _currentPoweredThrottleForce;
        private float _currentThrottleForce;

        [Header("Properties")] 
        [SerializeField] private GameObject model;
        [SerializeField] private Rigidbody rigidBody;
        [SerializeField] [Range(4, 6)] private int wingsCount = 6;
        [Tooltip("Auto-generated using predefined names if left empty.")]
        [SerializeField] private Transform[] wings = new Transform[6];

        [Header("Controller")]
        [SerializeField] private Vector2 rcControllerDeadzone = 
            new(.001f, .001f);
        [SerializeField] private Vector2 generalDeadzone = 
            new(.075f, .075f);
        [SerializeField] private Vector2 rcControllerMaxValue = 
            new(.9f, .9f);
        [SerializeField] private Vector2 generalMaxValue = 
            new(1f, 1f);

        void Awake()
        {
            _handler = new FsSm600Handler();
            _generalHandler = new GeneralInputHandler();
            _handlerFound = _handler.Device != null;
            _prevAxes = new SextupleAxesManager();
            _generalAxes = new SextupleAxesManager();

            rigidBody = GetComponent<Rigidbody>();
            
            UpdateAxisRange();

            xzPlane ??= transform.Find("XZPlane").GetComponent<HelperPlane>();

            // TODO: ACTUALLY IMPLEMENT POWER TOGGLE
            isPowered = true;
            
            if (wings[0] == null)
            {
                switch (wingsCount)
                {
                    case 6:
                        wings = new[]
                        {
                            model.transform.Find("Propeller CW F"),
                            model.transform.Find("Propeller CCW F"),
                            model.transform.Find("Propeller CCW M"),
                            model.transform.Find("Propeller CW M"),
                            model.transform.Find("Propeller CW B"),
                            model.transform.Find("Propeller CCW B"),
                        };
                        break;
                    case 4:
                        wings = new[]
                        {
                            model.transform.Find("Propeller CW F"),
                            model.transform.Find("Propeller CCW F"),
                            model.transform.Find("Propeller CCW B"),
                            model.transform.Find("Propeller CW B"),
                        };
                        break;
                    default:
                        throw new ArgumentException(
                            "Currently supports either 4 or 6 wings."    
                        );
                }
            }
            Array.Fill(_wingRotations, Quaternion.identity, 0, wings.Length);
            Array.Fill(_wingsXzDistances, 0f, 0, wings.Length);
            Array.Fill(_wingRotationsFromStrafe, Quaternion.identity, 0, wings.Length);
            
            _rotorCmd = new float[wingsCount];
            _rotorTarget = new float[wingsCount];
            _rotorTransition = new float[wingsCount];
        }

        public void UpdateAxisRange()
        {
            _handler.Axes.SetDeadzoneLeft(rcControllerDeadzone);
            _handler.Axes.SetDeadzoneRight(rcControllerDeadzone);
            _prevAxes.SetDeadzoneLeft(generalDeadzone);
            _prevAxes.SetDeadzoneRight(generalDeadzone);
            _generalAxes.SetDeadzoneLeft(generalDeadzone);
            _generalAxes.SetDeadzoneRight(generalDeadzone);
            
            _handler.Axes.SetMaxValueLeft(rcControllerMaxValue);
            _handler.Axes.SetMaxValueRight(rcControllerMaxValue);
            _prevAxes.SetMaxValueLeft(generalMaxValue);
            _prevAxes.SetMaxValueRight(generalMaxValue);
            _generalAxes.SetMaxValueLeft(generalMaxValue);
            _generalAxes.SetMaxValueRight(generalMaxValue);
        }

        void Update()
        {
            // for RC Controller
            if (_handlerFound)
            {
                _handlerFound = _handler.Poll();
                if (!_handlerFound) return;
                
                _handler.SendMessages(gameObject, _handler.Activeness);
                // this variable made for gamepad should obey when there's an rc controller
                _triggerToggle = Mathf.Approximately(_handler.Axes.Trigger, 1f);
            }
            else
            {
                _handlerPollTime += Time.deltaTime;
                if (_handlerPollTime >= _handlerPollTimer)
                {
                    _handlerPollTime = 0;
                    _handler.TryToConnect(
                        FsSm600Handler.Names, FsSm600Handler.ControlNames
                    );
                    _handlerFound = _handler.Device != null;
                }
                // when there's no rc controller, the variable simulates a toggle switch
                if (
                    Mathf.Approximately(_generalAxes.Trigger, 1f) &&
                    !Mathf.Approximately(_prevAxes.Trigger, 1f)
                )
                {
                    _triggerToggle = !_triggerToggle;
                }
            }
            
            // for general controllers
            
        }
        void FixedUpdate()
        {
            float fdt = Time.fixedDeltaTime;
            float alpha = 1f - Mathf.Exp(
                -Mathf.Max(1e-3f, commandSensitivity) * fdt
            );
            _worldUp = -Physics.gravity.normalized;
            float g = Physics.gravity.magnitude;
            
            if (isPowered)
            {
                _currentPoweredThrottleForce = baseThrottleForce;
                xzPlane.Tilt(_inputRight);
                CalculateDistanceFromWingsToXZ(wings, xzPlane.transform, _wingsXzDistances);
            }
            else
            {
                _currentPoweredThrottleForce = 0f;
                Array.Fill(_wingsXzDistances, 0f, 0, wingsCount);
            }

            // stick commands
            Vector4 stickCmd = new(
                _inputRight.x, // roll
                _inputRight.y, // pitch 
                _inputLeft.x, // yaw
                _inputLeft.y // lift
            );
            
            // limit roll/pitch before the body tilt ceiling
            Vector2 limitedRP = LimitHorizontalByTilt(new Vector2(
                stickCmd.x, stickCmd.y
            ));
            stickCmd.x = limitedRP.x;
            stickCmd.y = limitedRP.y;
            
            // hover force
            float tiltCos = Mathf.Clamp(
                Vector3.Dot(transform.up, _worldUp), tiltCompensationMinCos, 1f
            );
            float hoverForcePerRotor = autoHover ? 
                rigidBody.mass * g / Mathf.Max(1, wingsCount) / tiltCos : 
                baseThrottleForce;
            
            ApplyAutoLevel(g);

            for (int i = 0; i < wingsCount; i++)
            {
                // signs
                float rollS = RollSign(i); // -left / +right
                int pitchS = (int) WingPosAtOrder(i); // -front / 0 / +back
                int yawS = (int) WingDirAtOrder(i); // -ccw / +cw
                
                // mixing
                float liftT = liftMultiplier * stickCmd.w;
                float yawT = yawMix * stickCmd.z * yawS;
                float rollT = rollMix * stickCmd.x * rollS;
                float pitchT = pitchMix * stickCmd.y * pitchS;
                float strafeT = _wingsXzDistances[i];
                
                float targetForce =
                    hoverForcePerRotor +
                    throttleForce * (liftT + yawT + rollT + pitchT + strafeT);
                
                // power gating + clamp
                targetForce = Mathf.Clamp(
                    targetForce * (isPowered ? 1f : 0f), 0f, maxThrustPerRotor 
                );

                _rotorTarget[i] = targetForce;
                _rotorTransition[i] = Mathf.Lerp(
                    _rotorTransition[i], _rotorTarget[i], alpha
                );
                
                // thrust
                rigidBody.AddForceAtPosition(
                    transform.up * _rotorTransition[i],
                    wings[i].position,
                    ForceMode.Force
                );
                
                // reaction yaw torque
                float yawTorque = yawTorquePerThrust * _rotorTransition[i] * yawS * torqueMultiplier;
                rigidBody.AddRelativeTorque(
                    Vector3.up * yawTorque, ForceMode.Force
                );
                
                // base spin * strafe wooble
                GetDeltaWingRotation(
                    yawS,
                    _rotorTransition[i],
                    out Quaternion deltaSpin
                );
                _wingRotations[i] *= deltaSpin;
                wings[i].localRotation = 
                    _wingRotations[i] * _wingRotationsFromStrafe[i];
            }
        }

        // action for the attached game object
        public void ActionLeft(Vector2 v)
        {
            if (!isPowered) return;

            _inputLeft = v;
        }

        public void ActionRight(Vector2 v)
        {
            if (!isPowered) return;

            _inputRight = v;
        }

        /// <summary>
        /// 
        /// </summary>
        /// <param name="v">Received Aux value. Expected to be in the range of [0, 1].</param>
        public void ActionAux(float v)
        {
            _knobMix = v;
        }

        public void ActionTrigger(float v)
        {
            UpdateAxisRange();
            _triggerToggle = v > 0.5f;
        }
        
        /* Wings Position
         *
         * Case 6:
         * 0   1   2   3   4   5
         * F   F   M   M   B   B
         * CW  CCW CCW CW  CW  CCW
         * 1   1   0   0   -1  -1
         * 1   -1  -1  1   1   -1
         *
         * Mathf.Ceil(1 - i / 2)
         * i % 4 is 0 or 3 ? WingDir.CW : WingDir.CCW
         *
         * Case 4:
         * 0   1   2   3
         * F   F   B   B
         * CW  CCW CCW  CW
         * 1   1   -1  -1
         * 1   -1  -1  1
         *
         * Mathf.Ceil(1 - Mathf.Floor(i / 2) * 2)
         */
        
        private WingDir WingDirAtOrder(int i) => 
            i % 4 is 0 or 3 ? WingDir.CW : WingDir.CCW;
        private WingPos WingPosAtOrder(int i) => 
            (WingPos) Mathf.Ceil(1 - (i >> 1) * (4 - (wingsCount >> 1)));

        private float RollSign(int i)
        {
            Vector3 local = transform.InverseTransformPoint(wings[i].position);
            return Mathf.Abs(local.x) < 1e-3f ? 0f : Mathf.Sign(local.x);
        }

        /*
         * Torque -> Left Thumb -> Apply Up -> Spins Clockwise
         * Wing's Torque Exerted to its Body -> Right Thumb:
         *  Wing Spins Counterclockwise -> Rotates the body Clockwise -> Should be Applying Up
         */
        
        // drone animation (unit)

        /// <summary>
        /// Get the next small rotation of a wing in `d`irection by `f`orce to its `q`uaternion variable.
        /// This is purely for cosmetic purposes.
        /// </summary>
        /// <param name="d">The direction of the rotation. 1 for CW, -1 for CCW.</param>
        /// <param name="f">Amount of the force to apply.</param>
        /// <param name="q">Variable to store the rotation of the wing.</param>
        private void GetDeltaWingRotation(
            float d,
            float f,
            out Quaternion q
        )
        {
            float degPerSec = 
                wingRotationMultiplier * (_currentPoweredThrottleForce + f);
            float deltaDeg = d * degPerSec * Time.fixedDeltaTime;
            q = Quaternion.Euler(0, deltaDeg, 0);
        }
        /// <summary>
        /// Get the next small rotation of a `w`ing in `d`irection by `f`orce to `q`uaternion variable,
        /// and apply torque and force to the `b`ody the wing is attached to.
        /// </summary>
        /// <param name="w">Transform of the wing to rotate.</param>
        /// <param name="d">The direction of the rotation. 1 for CW, -1 for CCW.</param>
        /// <param name="f">Amount of the force to apply.</param>
        /// <param name="q">Variable to store the rotation of the wing.</param>
        /// <param name="b">Rigidbody of the body the wing is attached to.</param>
        private void GetDeltaWingRotationAndAddTorque(
            Transform w,
            float d,
            float f,
            out Quaternion q,
            Rigidbody b
        )
        {
            GetDeltaWingRotation(d, f, out q);
            
            b.AddRelativeTorque(
                -d * Vector3.up * (_currentPoweredThrottleForce + f),
                ForceMode.Force
            );
            b.AddForceAtPosition(
                Vector3.up * (_currentPoweredThrottleForce + f),
                w.position,
                ForceMode.Force
            );
        }

        private void CalculateDistanceFromWingsToXZ(
            // ReSharper disable once InconsistentNaming
            IList<Transform> wingPos, Transform XZ, IList<float> dist
        )
        {
            Vector3 p0 = XZ.position;
            Vector3 n = XZ.up;
            float denom = Vector3.Dot(transform.up, n);

            for (int i = 0; i < wingPos.Count; i++)
            {
                Vector3 wp = wingPos[i].position;
                dist[i] = Mathf.Abs(denom) < 1e-6f
                    ? 0f
                    : Vector3.Dot(p0 - wp, n) / denom;

                dist[i] *= strafeRate;
                dist[i] = Mathf.Clamp(
                    dist[i],
                    -strafeRate,
                    strafeRate
                );
            }
        }
        
        /// <summary>
        /// Keeps the drone level in roll/pitch when there's no horizontal input.
        /// </summary>
        /// <param name="g">gravitational force</param>
        private void ApplyAutoLevel(float g)
        {
            if (!autoHover || !isPowered) return;

            // How much the player is actively tilting (right stick)
            float userTilt = Mathf.Clamp01(new Vector2(_inputRight.x, _inputRight.y).magnitude);
            float levelScale = 1f - userTilt; // stronger leveling when the stick is released
            if (levelScale <= 1e-4f) return;

            // Tilt axis to bring transform.up -> world up
            Vector3 tiltAxis = Vector3.Cross(transform.up, _worldUp);
            float sinAngle = tiltAxis.magnitude;
            if (sinAngle < 1e-4f) return;

            Vector3 axis = tiltAxis / sinAngle;
            float angleErr = Mathf.Asin(Mathf.Clamp(sinAngle, 0f, 1f)); // radians

            // Gain from existing knobs (no new fields)
            float kp = torqueMultiplier * (rigidBody.mass * g) * 0.5f;
            float kd = torqueMultiplier * commandSensitivity * 0.25f;

            // Proportional correction (only roll/pitch)
            Vector3 corrective = axis * (angleErr * kp) * levelScale;

            // Dampen roll/pitch angular velocity (ignore yaw)
            Vector3 ang = rigidBody.angularVelocity;
            Vector3 angNoYaw = ang - Vector3.Project(ang, _worldUp);
            Vector3 damping = -kd * angNoYaw;

            rigidBody.AddTorque(corrective + damping, ForceMode.Force);
        }

        
        /// <summary>
        /// Scale horizontal (roll/pitch) input so we never push past the body tilt ceiling.
        /// </summary>
        /// <param name="cmdRP"></param>
        /// <returns></returns>
        private Vector2 LimitHorizontalByTilt(Vector2 cmdRP)
        {
            // Current body tilt
            float cosUp = Mathf.Clamp(Vector3.Dot(transform.up, _worldUp), -1f, 1f);
            float tiltRad = Mathf.Acos(cosUp);
            float maxRad  = Mathf.Max(1e-4f, maxTiltDegree * Mathf.Deg2Rad);

            // Below the ceiling? No gating.
            if (tiltRad < maxRad) return cmdRP;

            // Axis that would LEVEL the craft (rotate transform.up -> _worldUp)
            Vector3 axis = Vector3.Cross(transform.up, _worldUp);
            float m = axis.magnitude;
            if (m < 1e-4f) return cmdRP;
            axis /= m;

            // In roll/pitch space: direction that would INCREASE tilt (opposite of leveling axis)
            Vector2 incDirRP = new Vector2(
                Vector3.Dot(-axis, transform.forward), // roll(+)
                Vector3.Dot(-axis, transform.right)    // pitch(+)
            );
            float incMag = incDirRP.magnitude;
            if (incMag < 1e-4f) return cmdRP;

            // If the stick is pushing in the "increase tilt" direction, fade it out.
            Vector2 incN = incDirRP / incMag;
            float cmdMag = cmdRP.magnitude;
            if (cmdMag < 1e-4f) return cmdRP;

            float alignment = Vector2.Dot(cmdRP / cmdMag, incN); // [-1..1]
            if (alignment <= 0f) return cmdRP; // trying to level → allow fully

            // Fully aligned with "increase tilt" → 0, orthogonal → 1
            float scale = 1f - alignment;
            return cmdRP * scale;
        }


        // event handling for generic controllers (gamepads and keyboards + mouses)
        public void OnLeft(InputValue v)
        {
            _prevAxes.Left = _generalAxes.Left;
            _generalAxes.Left = _generalHandler.OnLeft(v);
            ActionLeft(_generalAxes.Left);
        }
        public void OnRight(InputValue v)
        {
            _prevAxes.Right = _generalAxes.Right;
            _generalAxes.Right = _generalHandler.OnRight(v);
            ActionRight(_generalAxes.Right);
        }
        public void OnAux(InputValue v)
        {
            _prevAxes.Aux = _generalAxes.Aux;
            _generalAxes.Aux = _generalHandler.OnAux(v);
            ActionAux(_generalAxes.Aux);
        }
        public void OnTrigger(InputValue v)
        {
            _prevAxes.Trigger = _generalAxes.Trigger;
            _generalAxes.Trigger = _generalHandler.OnTrigger(v);
            ActionTrigger(_generalAxes.Trigger);
        }
        // event handling for RC Controllers
        public void OnLeft(int _) => 
            ActionLeft(_handler.OnLeft());
        public void OnRight(int _) => 
            ActionRight(_handler.OnRight());
        public void OnAux(int _) => 
            ActionAux(_handler.OnAux());
        public void OnTrigger(int _) => 
            ActionTrigger(_handler.OnTrigger());
    }
}
