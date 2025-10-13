using System;
using System.Collections.Generic;
using ControlHandler;
using UnityEngine;
using UnityEngine.InputSystem;

namespace DroneMovement
{
    public enum WingDir { CCW = -1, CW = 1 }
    public enum WingPos { Front = 0, Middle = 1, Back = 2 }
     
    [RequireComponent(typeof(Rigidbody), typeof(PlayerInput))]
    public class DroneController : MonoBehaviour
    {
        private const int InspectorSpace = 14;
        internal FsSm600Handler Handler;
        internal GeneralInputHandler GeneralHandler;
        internal bool HandlerFound;
        internal float HandlerPollTime;
        internal readonly float HandlerPollTimer = 3f;
        internal SextupleAxesManager PrevAxes;
        internal SextupleAxesManager GeneralAxes;
        internal bool TriggerToggle; // from OnTrigger
        internal float KnobMix; // from OnAux

        private readonly Quaternion[] _wingRotations = new Quaternion[6];
        private readonly float[] _wingsXzDistances = new float[6];
        private readonly Quaternion[] _wingRotationsFromStrafe = new Quaternion[6];

        [Header("Configuration")]
        [SerializeField] private bool isPowered;
        [SerializeField] private bool autoHover = true;
        [Tooltip("N (kg·m/s²), applies 'mass * gravity' instead when Auto Hover is on.")]
        [SerializeField] [Min(0)] private float baseThrottleForce = 49.05f;
        [Tooltip("N (kg·m/s²)")]
        [SerializeField] [Min(0)] private float appliedThrottleForce = 25f;

        [Header("Proportional-Derivative Controller")] 
        [Tooltip("Hz, How \"fast\" rotations feel.")] 
        [SerializeField] [Min(.1f)] private float attitudeBandwidth = 3f;
        [Tooltip(".7~1.0, How much it \"resists\" oscilation.")]
        [SerializeField] [Range(.2f, 2f)] private float attitudeDamping = .55f;
        private const float MaxNm = 1e6f;

        [Header("Physical Property")]
        [SerializeField] [Min(0)] private float rotationalMultiplier = 20f; // for movement
        [Tooltip("deg/s")]
        [SerializeField] [Min(0)] private float yawRate = 90f;
        [Space(InspectorSpace)]
        [SerializeField] [Min(0)] private float strafeRate = 1f;
        [Tooltip("An invisible plane sitting on the wings attitude in drone game object, tilts as xz movement input is received.")]
        [SerializeField] HelperPlane xzPlane;
        [Tooltip("deg, how much the helper plane can tilt.")]
        [SerializeField] [Min(0)] private float maxTiltDegree = 30f;
        [Tooltip("deg")]
        [SerializeField] [Range(0, 1f)] private float tiltCompensationMinCos = .35f;
        [Space(InspectorSpace)]
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

        private Vector3 _movV = Vector3.zero;
        private Vector3 _currentUp;
        private Vector3 _worldUp;
        private Quaternion _rotQ = Quaternion.identity;
        private float _yawInput;
        private float _targetYaw;

        void Awake()
        {
            Handler = new FsSm600Handler();
            GeneralHandler = new GeneralInputHandler();
            HandlerFound = Handler.Device != null;
            PrevAxes = new SextupleAxesManager();
            GeneralAxes = new SextupleAxesManager();

            rigidBody = GetComponent<Rigidbody>();
            
            UpdateAxisRange();

            xzPlane ??= transform.Find("XZPlane").GetComponent<HelperPlane>();

            // TODO: ACTUALLY IMPLEMENT POWER TOGGLE
            isPowered = false;
            
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
        }

        public void UpdateAxisRange()
        {
            Handler.Axes.SetDeadzoneLeft(rcControllerDeadzone);
            Handler.Axes.SetDeadzoneRight(rcControllerDeadzone);
            PrevAxes.SetDeadzoneLeft(generalDeadzone);
            PrevAxes.SetDeadzoneRight(generalDeadzone);
            GeneralAxes.SetDeadzoneLeft(generalDeadzone);
            GeneralAxes.SetDeadzoneRight(generalDeadzone);
            
            Handler.Axes.SetMaxValueLeft(rcControllerMaxValue);
            Handler.Axes.SetMaxValueRight(rcControllerMaxValue);
            PrevAxes.SetMaxValueLeft(generalMaxValue);
            PrevAxes.SetMaxValueRight(generalMaxValue);
            GeneralAxes.SetMaxValueLeft(generalMaxValue);
            GeneralAxes.SetMaxValueRight(generalMaxValue);
        }

        void Update()
        {
            // for RC Controller
            if (HandlerFound)
            {
                HandlerFound = Handler.Poll();
                if (!HandlerFound) return;
                
                Handler.SendMessages(gameObject, Handler.Activeness);
                // this variable made for gamepad should obey when there's an rc controller
                TriggerToggle = Mathf.Approximately(Handler.Axes.Trigger, 1f);
            }
            else
            {
                HandlerPollTime += Time.deltaTime;
                if (HandlerPollTime >= HandlerPollTimer)
                {
                    HandlerPollTime = 0;
                    Handler.TryToConnect(
                        FsSm600Handler.Names, FsSm600Handler.ControlNames
                    );
                    HandlerFound = Handler.Device != null;
                }
                // when there's no rc controller, the variable simulates a toggle switch
                if (
                    Mathf.Approximately(GeneralAxes.Trigger, 1f) &&
                    !Mathf.Approximately(PrevAxes.Trigger, 1f)
                )
                {
                    TriggerToggle = !TriggerToggle;
                }
            }
            
            // for general controllers
            
        }
        void FixedUpdate()
        {
            float fdt = Time.fixedDeltaTime;
            _worldUp = -Physics.gravity.normalized;
            baseThrottleForce = autoHover ? 
                rigidBody.mass * Physics.gravity.magnitude :
                baseThrottleForce;
            
            // update powered state
            _currentPoweredThrottleForce = Mathf.Lerp(
                _currentPoweredThrottleForce, 
                isPowered ? baseThrottleForce : 0, 
                fdt
            );
            
            if (isPowered)
            {
                // --- LIFT ---
                _currentThrottleForce = 
                    _currentPoweredThrottleForce + 
                    appliedThrottleForce * _movV.y;
                
                // tilt compensation to vertical component of thrust
                float cosTilt = Mathf.Clamp(
                    Vector3.Dot(transform.up, _worldUp),
                    tiltCompensationMinCos,
                    1f
                );
                float tiltCompensation = 1f / cosTilt;

                _currentThrottleForce *= tiltCompensation;
                
                // vertical velocity damping
                float velocityV = 
                    Vector3.Dot(rigidBody.linearVelocity, _worldUp);
                float vertDampForce = 
                    -velocityV * rigidBody.linearDamping * rigidBody.mass;
                
                // apply lift force
                rigidBody.AddForce(
                    transform.up * _currentThrottleForce +
                    _worldUp * vertDampForce,
                    ForceMode.Force
                );

                // --- attitude ---
                // yaw target (hold when stick returns to 0)
                _targetYaw += _yawInput * yawRate * fdt;

                // yaw target * pitch/roll target
                Quaternion targetOrientation =
                    Quaternion.AngleAxis(_targetYaw, Vector3.up) * _rotQ;

                // get torque toward target
                Quaternion qError =
                    Quaternion.Inverse(rigidBody.rotation) * targetOrientation;
                qError.ToAngleAxis(
                    out float errorDeg, out Vector3 errorAxis
                );
                if (errorDeg > 180f)
                    errorDeg -= 360f;
                if (
                    Mathf.Abs(errorDeg) >= 1e-3f &&
                    float.IsFinite(errorAxis.sqrMagnitude)
                )
                {
                    /* - PD = Proportional-Derivative Controller
                     * Proportional term
                     *   - "spring" pulling towards the target angle
                     *   - larger error -> more torque
                     *   => _axis.normalized * (errorRad * kp)
                     * Derivative term
                     *   - "damper" resisting angular velocity
                     *   - slows down motion before overshoot
                     *   => -angularVelocity * kd
                     * 
                     * torque = Kp * rotation_error - Kd * angularVelocity
                     * 
                     * P makes it rotate towards, D stops it.
                     * Too low D gets it wobble, too high sluggy.
                     * 
                     * - current angular velocity in body frame
                     * Vector3 omegaBody = transform.InverseTransformDirection(
                     *     rigidBody.angularVelocity
                     * );
                     * 
                     * - Body -> principal inertia frame
                     * Quaternion qI = rigidBody.inertiaTensorRotation;
                     * Vector3 omegaI = Quaternion.Inverse(qI) * omegaBody;
                     * Vector3 axisI  = Quaternion.Inverse(qI) * axisB;
                     *
                     * - per-axis inertia (diagonal in principal coords
                     * Vector3 I = rigidBody.inertiaTensor;
                     * 
                     * - wn - natural frequency (rad/s)
                     *   "how fast" it tries to correct an angular error
                     * - z - damping ratio (dimensionless)
                     *   "how much" it resists oscillation
                     *
                     * Kp - proportional torque gain
                     * Kd - damping torque gain
                     *
                     * Kp = ωₙ²
                     * Kd = 2 ζ ωₙ
                     */
                    
                    // Map body -> principal inertia frame
                    Quaternion itr = rigidBody.inertiaTensorRotation;
                    Vector3 omegaItr = Quaternion.Inverse(itr) * transform
                        .InverseTransformDirection(
                            rigidBody.angularVelocity
                        );
                    Vector3 axisItr = Quaternion.Inverse(itr) * 
                        errorAxis.normalized;
                    
                    // per-axis inertia
                    Vector3 it = rigidBody.inertiaTensor;
                    
                    // ωₙ
                    float natFreq = 2f * Mathf.PI * attitudeBandwidth;
                    // ζ = attitudeDamping;
                    
                    // Per-axis PD in terms of desired angular acceleration
                    // torque = Kp*rotation_error - Kd*angularVelocity
                    // Kp = wn * wn, Kd = 2 * z * wn
                    float pTorqueGain = natFreq * natFreq;
                    float dTorqueGain = 2f * attitudeDamping * natFreq;
                    
                    // Project error onto each principal axis
                    Vector3 alphaItr =
                        axisItr * (pTorqueGain * (Mathf.Deg2Rad * errorDeg)) -
                        dTorqueGain * omegaItr;
                    
                    // torque in principal frame τ = I ⊙ α
                    Vector3 tauItr = Vector3.Scale(it, alphaItr);
                    
                    // principal -> body -> world
                    Vector3 tau = itr * tauItr;
                    Vector3 tauWorld = transform.TransformDirection(tau);

                    if (tauWorld.sqrMagnitude > MaxNm * MaxNm)
                        tauWorld = tauWorld.normalized * MaxNm;
                    
                    rigidBody.AddTorque(
                        tauWorld, 
                        ForceMode.Force
                    );
                }
            }
            
            // compute wing rotations
            for (int i = 0; i < wingsCount; i++)
            {
                float d = (float) WingDirAtOrder(i);
                ChangeWingRotation(
                    d, 
                    isPowered ? d : 0, 
                    out _wingRotations[i]
                );
            }
            if (isPowered)
                CalculateDistanceFromWingsToXZ(wings, xzPlane.transform, _wingsXzDistances);
            else
                Array.Fill(_wingsXzDistances, 0f, 0, wings.Length);
            
            // apply wing rotations
            for (int i = 0; i < wingsCount; i++)
            {
                Quaternion deltaWingRotation = Quaternion.Slerp(
                    wings[i].localRotation,
                    isPowered ? 
                        _wingRotationsFromStrafe[i] * _wingRotations[i] : 
                        Quaternion.identity,
                    _wingsXzDistances[i] * fdt
                );

                wings[i].localRotation = deltaWingRotation;
            }
            /*/
            Debug.Log($"{_wingRotationsFromStrafe[0]*_wingRotations[0]}, {_wingRotationsFromStrafe[1]*_wingRotations[1]}, {_wingRotationsFromStrafe[2]*_wingRotations[2]}, {_wingRotationsFromStrafe[3]*_wingRotations[3]}, {_wingRotationsFromStrafe[4]*_wingRotations[4]}, {_wingRotationsFromStrafe[5]*_wingRotations[5]}");
            /*/
            Debug.Log($"{wings[0].localRotation.y:F2}, {wings[1].localRotation.y:F2}, {wings[2].localRotation.y:F2}, {wings[3].localRotation.y:F2}, {wings[4].localRotation.y:F2}, {wings[5].localRotation.y:F2}");
            //*/
        }

        // action for the attached game object
        public void ActionLeft(Vector2 v)
        {
            if (!isPowered) return;
            
            _movV.y = Mathf.Clamp(v.y, -1f, 1f);
            _yawInput = Mathf.Clamp(v.x, -1f, 1f);

            for (int i = 0; i < wingsCount; i++)
            {
                float d = (float)WingDirAtOrder(i);
                ChangeWingRotation(
                    d,
                    d * appliedThrottleForce * v.x,
                    out _wingRotations[i]
                );
            }
        }

        public void ActionRight(Vector2 v)
        {
            if (!isPowered) return;
            
            xzPlane.Tilt(v);
            MovementXZ();
            
            _rotQ = Quaternion.Euler(
                maxTiltDegree * v.y,
                0f,
                maxTiltDegree * -v.x
            );
        }

        /// <summary>
        /// 
        /// </summary>
        /// <param name="v">Received Aux value. Expected to be in the range of [0, 1].</param>
        public void ActionAux(float v)
        {
            KnobMix = v;
        }

        public void ActionTrigger(float v)
        {
            UpdateAxisRange();
            TriggerToggle = v > 0.5f;
        }
        
        /* Wings Position
         *
         * Case 6:
         * 0   1   2   3   4   5
         * F   F   M   M   B   B
         * CW  CCW CCW CW  CW  CCW
         * 0   0   1   1   2   2
         * 1   -1  -1  1   1   -1
         *
         * 2 * Pos + Dir
         * i % 4 is 0 or 3 ? WingDir.CW : WingDir.CCW
         *
         * Case 4:
         * 0   1   2   3
         * F   F   B   B
         * CW  CCW CCW  CW
         * 0   0   2   2
         * 1   -1  -1  1
         */
        
        private WingDir WingDirAtOrder(int i) => 
            i % 4 is 0 or 3 ? WingDir.CW : WingDir.CCW;

        // drone animation (complex)
        private void MovementRotate(float v)
        {
            for (int i = 0; i < wingsCount; i++)
            {
                float d = (float) WingDirAtOrder(i);
                ChangeWingRotationAndAddTorque(
                    wings[i], 
                    d, 
                    d * appliedThrottleForce * v, 
                    out _wingRotations[i], 
                    rigidBody
                );
            }
        }
        private void MovementY(float v)
        {
            for (int i = 0; i < wingsCount; i++)
            {
                float d = 1f;
                ChangeWingRotationAndAddTorque(
                    wings[i],
                    d,
                    d * appliedThrottleForce * v, 
                    out _wingRotations[i], 
                    rigidBody
                );
            }
        }
        private void MovementXZ()
        {
            for (int i = 0; i < wingsCount; i++)
            {
                float d = (float) WingDirAtOrder(i);
                ChangeWingRotation(
                    d,
                    d * appliedThrottleForce * _wingsXzDistances[i],
                    out _wingRotationsFromStrafe[i]
                );
            }
        }

        /*
         * Torque -> Left Thumb -> Apply Up -> Spins Clockwise
         * Wing's Torque Exerted to its Body -> Right Thumb:
         *  Wing Spins Counterclockwise -> Rotates the body Clockwise -> Should be Applying Up
         */
        
        // drone animation (unit)

        /// <summary>
        /// Change rotation of a wing in `d`irection by `f`orce to its `q`uaternion variable.
        /// This is purely for cosmetic purposes.
        /// </summary>
        /// <param name="d">The direction of the rotation. 1 for CW, -1 for CCW.</param>
        /// <param name="f">Amount of the force to apply.</param>
        /// <param name="q">Variable to store the rotation of the wing.</param>
        private void ChangeWingRotation(
            float d,
            float f,
            out Quaternion q
        )
        {
            q = Quaternion.Euler(
                0, 
                d * wingRotationMultiplier * (_currentPoweredThrottleForce + f),
                0
            );
        }
        /// <summary>
        /// Change rotation of a `w`ing in `d`irection by `f`orce to `q`uaternion variable,
        /// and apply torque and force to the `b`ody the wing is attached to.
        /// </summary>
        /// <param name="w">Transform of the wing to rotate.</param>
        /// <param name="d">The direction of the rotation. 1 for CW, -1 for CCW.</param>
        /// <param name="f">Amount of the force to apply.</param>
        /// <param name="q">Variable to store the rotation of the wing.</param>
        /// <param name="b">Rigidbody of the body the wing is attached to.</param>
        private void ChangeWingRotationAndAddTorque(
            Transform w,
            float d,
            float f,
            out Quaternion q,
            Rigidbody b
        )
        {
            ChangeWingRotation(d, f, out q);
            
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

        // event handling for generic controllers (gamepads and keyboards + mouses)
        public void OnLeft(InputValue v)
        {
            PrevAxes.Left = GeneralAxes.Left;
            GeneralAxes.Left = GeneralHandler.OnLeft(v);
            ActionLeft(GeneralAxes.Left);
        }
        public void OnRight(InputValue v)
        {
            PrevAxes.Right = GeneralAxes.Right;
            GeneralAxes.Right = GeneralHandler.OnRight(v);
            ActionRight(GeneralAxes.Right);
        }
        public void OnAux(InputValue v)
        {
            PrevAxes.Aux = GeneralAxes.Aux;
            GeneralAxes.Aux = GeneralHandler.OnAux(v);
            ActionAux(GeneralAxes.Aux);
        }
        public void OnTrigger(InputValue v)
        {
            PrevAxes.Trigger = GeneralAxes.Trigger;
            GeneralAxes.Trigger = GeneralHandler.OnTrigger(v);
            ActionTrigger(GeneralAxes.Trigger);
        }
        // event handling for RC Controllers
        public void OnLeft(int _) => 
            ActionLeft(Handler.OnLeft());
        public void OnRight(int _) => 
            ActionRight(Handler.OnRight());
        public void OnAux(int _) => 
            ActionAux(Handler.OnAux());
        public void OnTrigger(int _) => 
            ActionTrigger(Handler.OnTrigger());
    }
}
