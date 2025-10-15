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

        [Header("Configuration")] [SerializeField]
        private bool isPowered;

        [SerializeField] private bool autoHover = true;

        [Tooltip("N (kg·m/s²), applies 'mass * gravity' instead when Auto Hover is on.")] [SerializeField, Min(0)]
        private float baseThrottleForce = 49.05f;

        [Tooltip("N (kg·m/s²)")] [SerializeField, Min(0)]
        private float appliedThrottleForce = 25f;

        [Header("Rotor Model")] [Tooltip("N·m/N. Yaw reaction torque per 1N of thrust.")] [SerializeField, Min(0)]
        private float yawDragPerNewton = .02f;

        [Tooltip("N. Lower/upper clamp for per-wing thrust.")] [SerializeField]
        private Vector2 thrustClamp = new Vector2(0, 1e6f);

        private float[] _wingThrust; // N
        private Vector3[] _lever; // r_i x up (world)
        private Vector3 _wingThrustDirection;

        [Header("Rotor Visual")] 
        [SerializeField, Min(0)]
        private float wingRotationMultiplier = 16f; // for cosmetic wing rotation
        [Tooltip("s. How fast visuals chase target speed wheen powered.")] 
        [SerializeField, Min(0)]
        private float spinResponseTime = .1f;
        [Tooltip("s. How fast visuals wind down when unpowered.")] 
        [SerializeField, Min(0)]
        private float windDownTime = 1.5f;
        [Tooltip("deg/s. Minimum visual spin when powered.")] 
        [SerializeField, Min(0)]
        private float minSpin = 0.15f;
        [Tooltip("deg/s. Maximum visual spin when powered.")] 
        [SerializeField, Min(0)]
        private float maxSpin = 360f * 60f;

        private float[] _currentSpinDegPerSec;

        [Header("Proportional-Derivative Controller")] 
        [Tooltip("Hz, How \"fast\" rotations feel.")] 
        [SerializeField] [Min(.1f)] private float attitudeBandwidth = 3f;
        [Tooltip(".7~1.0, How much it \"resists\" oscilation.")]
        [SerializeField] [Range(.2f, 2f)] private float attitudeDamping = .55f;
        private const float MaxNm = 1e6f;

        [Header("Physical Property")]
        [Tooltip("deg/s")]
        [SerializeField, Min(0)] private float yawRate = 90f;
        [Space(InspectorSpace)]
        [SerializeField, Min(0)] private float strafeRate = 1f;
        [Tooltip("An invisible plane sitting on the wings attitude in drone game object, tilts as xz movement input is received.")]
        [SerializeField] HelperPlane xzPlane;
        [Tooltip("deg, how much the helper plane can tilt.")]
        [SerializeField, Min(0)] private float maxTiltDegree = 30f;
        [Tooltip("deg")]
        [SerializeField] [Range(0, 1f)] private float tiltCompensationMinCos = .35f;
        [Space(InspectorSpace)]
        private float _currentPoweredThrottleForce;
        private float _currentThrottleForce;

        [Header("Properties")] 
        [SerializeField] private GameObject model;
        [SerializeField] private Rigidbody rigidBody;
        [SerializeField] [Range(4, 6)] private int wingsCount = 6;
        [Tooltip("Auto-generated using predefined names if left empty.")]
        [SerializeField] private Transform[] wings = new Transform[6];
        [Tooltip("How many Conjugate-Gradient iterations to take per physics step.")]
        [SerializeField, Range(1, 8)] private int conjugateGradientIterations = 3;

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
            _wingThrust = new float[wingsCount];
            _lever = new Vector3[wingsCount];
            _currentSpinDegPerSec = new float[wingsCount];
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
            // for general controllers
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
        }

        void FixedUpdate()
        {
            float fdt = Time.fixedDeltaTime;
            _worldUp = -Physics.gravity.normalized;
            _wingThrustDirection = transform.up;

            baseThrottleForce = autoHover ? rigidBody.mass * Physics.gravity.magnitude : baseThrottleForce;

            // update powered state
            _currentPoweredThrottleForce = Mathf.Lerp(
                _currentPoweredThrottleForce,
                isPowered ? baseThrottleForce : 0,
                fdt
            );

            // PD to get desired torque
            Vector3 desiredTauWorld = Vector3.zero;

            if (isPowered)
            {
                // --- LIFT ---
                _currentThrottleForce = Mathf.Max(
                    0f,
                    _currentPoweredThrottleForce +
                    appliedThrottleForce * _movV.y
                );

                // tilt compensation to vertical component of thrust
                float cosTilt = Mathf.Clamp(
                    Vector3.Dot(_wingThrustDirection, _worldUp),
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

                _currentThrottleForce += vertDampForce;

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
                    desiredTauWorld = transform.TransformDirection(itr * tauItr);

                    if (desiredTauWorld.sqrMagnitude > MaxNm * MaxNm)
                        desiredTauWorld = desiredTauWorld.normalized * MaxNm;
                }
            }

            // allocate thrust to wings and apply forces/torques
            if (isPowered && _currentThrottleForce > 0f)
                AllocateAndApplyRotorForces(_currentThrottleForce, desiredTauWorld);
            // zero thrust -> gently spin down visually
            else
                for (int i = 0; i < wingsCount; i++)
                    _wingThrust[i] = 0f;

            // calculate wing rotations from strafes
            if (isPowered)
                CalculateDistanceFromWingsToXZ(wings, xzPlane.transform, _wingsXzDistances);
            else
                Array.Fill(_wingsXzDistances, 0f, 0, wings.Length);
            
            // wing spin driven by per-wing force
            UpdateWingSpin(fdt);

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

        private void UpdateWingSpin(float dt)
        {
            float alphaOn =
                1f - Mathf.Exp(-dt / Mathf.Max(1e-4f, spinResponseTime));
            float alphaOff =
                1f - Mathf.Exp(-dt / Mathf.Max(1e-4f, windDownTime));

            float avgF = 0f;
            for (int i = 0; i < wingsCount; i++) 
                avgF += Mathf.Max(0f, _wingThrust[i]);
            avgF /= Mathf.Max(1, wingsCount);
            float floorF = minSpin * avgF;
            
            for (int i = 0; i < wingsCount; i++)
            {
                float d = (float) WingDirAtOrder(i);
                float iF = Mathf.Max(0f, _wingThrust[i]);
                float iFVisual = isPowered ? Mathf.Max(iF, floorF) : 0f;
                
                float targetDegPerSec = isPowered ? 
                    d * wingRotationMultiplier * iFVisual
                    : 0f;
                
                float a = isPowered ? alphaOn : alphaOff;
                _currentSpinDegPerSec[i] = Mathf.Lerp(
                    _currentSpinDegPerSec[i], targetDegPerSec, a
                );

                _currentSpinDegPerSec[i] = Mathf.Clamp(
                    _currentSpinDegPerSec[i], 
                    -maxSpin, 
                    maxSpin
                );

                float stepDeg = _currentSpinDegPerSec[i] * dt;
                wings[i].localRotation *= 
                    Quaternion.AngleAxis(stepDeg, Vector3.up);
            }
        }

        /*
         * Torque -> Left Thumb -> Apply Up -> Spins Clockwise
         * Wing's Torque Exerted to its Body -> Right Thumb:
         *  Wing Spins Counterclockwise -> Rotates the body Clockwise -> Should be Applying Up
         */
        
        // drone animation (unit)

        /// <summary>
        /// Get a small change of a wing in `d`irection by `f`orce to its `q`uaternion variable over small time of `dt`.
        /// This is purely for cosmetic purposes.
        /// </summary>
        /// <param name="d">The direction of the rotation. 1 for CW, -1 for CCW.</param>
        /// <param name="f">Amount of the force to apply.</param>
        /// <param name="q">Variable to store the rotation of the wing.</param>
        /// <param name="dt">small time</param>
        private void GetDeltaWingRotation(
            float d,
            float f,
            out Quaternion q,
            float dt
        )
        {
            q = Quaternion.Euler(
                0,
                d * wingRotationMultiplier * 
                (_currentPoweredThrottleForce + f) * dt,
                0
            );
        }
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
        /// Map PD torque plus total thrust to per-wing thrust and apply them.
        /// </summary>
        /// <param name="totalThrustN"></param>
        /// <param name="desiredTauWorld"></param>
        private void AllocateAndApplyRotorForces(float totalThrustN, Vector3 desiredTauWorld)
        {
            // Build lever arms (r_i × _wingThrustDirection) in world space
            for (int i = 0; i < wingsCount; i++)
            {
                Vector3 r = wings[i].position - rigidBody.worldCenterOfMass;
                _lever[i] = Vector3.Cross(r, _wingThrustDirection);
            }

            // Split torque into "tilt torque" (roll/pitch) and "yaw torque"
            float tauYaw = Vector3.Dot(desiredTauWorld, _wingThrustDirection);
            Vector3 tauTilt = desiredTauWorld - tauYaw * _wingThrustDirection;

            // Solve least-squares ΔF: A ΔF ~ tauTilt  -> ΔF = A^T (A A^T)^-1 tauTilt
            Vector3 v = SolveGVEqualsTau(tauTilt, _lever);
            float basePerWing = totalThrustN / wingsCount;
            for (int i = 0; i < wingsCount; i++)
                _wingThrust[i] = basePerWing + Vector3.Dot(_lever[i], v);

            // Add yaw via reaction torque; choose c so sum of reaction torques ~ tauYaw.
            // Reaction from wing i: τ_i_yaw ~ -dir_i * yawDragPerNewton * F_i * _wingThrustDirection.
            // Using zero-sum mixing: add c*dir_i to each F_i -> sum yaw = -yawDrag*N*c.
            float cYaw = (-tauYaw) / (yawDragPerNewton * wingsCount);
            for (int i = 0; i < wingsCount; i++)
                _wingThrust[i] += cYaw * (float)WingDirAtOrder(i);

            // Clamp, keep total thrust roughly constant
            float sum = 0f;
            for (int i = 0; i < wingsCount; i++)
            {
                _wingThrust[i] = Mathf.Clamp(
                    _wingThrust[i], thrustClamp.x, thrustClamp.y
                );
                sum += _wingThrust[i];
            }
            // Renormalize to preserve totalThrustN
            float renorm = (sum > 1e-6f) ? totalThrustN / sum : 0f;
            for (int i = 0; i < wingsCount; i++) _wingThrust[i] *= renorm;

            // Apply the forces and yaw reaction torques; spin visuals from thrust.
            for (int i = 0; i < wingsCount; i++)
            {
                float f = _wingThrust[i];
                int dir = (int)WingDirAtOrder(i); // +1 CW, -1 CCW

                rigidBody.AddForceAtPosition(
                    _wingThrustDirection * f, wings[i].position, ForceMode.Force
                );
                rigidBody.AddTorque(
                    -dir * yawDragPerNewton * f * _wingThrustDirection, ForceMode.Force
                );

                // cosmetic spin from thrust
                ChangeWingRotation(dir, f, out _wingRotations[i]);
            }
        }

        /// <summary>
        /// y = G(x) = (Σ li li^T) x
        /// </summary>
        /// <param name="x"></param>
        /// <param name="lever"></param>
        /// <returns></returns>
        private static Vector3 GMul(Vector3 x, IList<Vector3> lever)
        {
            Vector3 y = Vector3.zero;
            for (int i = 0; i < lever.Count; i++)
                y += lever[i] * Vector3.Dot(lever[i], x);
            return y;
        }

        /// <summary>
        /// Solve G v = tau using Conjugate Gradient (G is SPD).
        /// </summary>
        /// <param name="tau"></param>
        /// <param name="lever"></param>
        /// <returns></returns>
        // ReSharper disable once InconsistentNaming
        private Vector3 SolveGVEqualsTau(Vector3 tau, IList<Vector3> lever)
        {
            Vector3 v = Vector3.zero;
            Vector3 r = tau - GMul(v, lever);
            Vector3 p = r;
            float rs = Vector3.Dot(r, r);

            for (int k = 0; k < conjugateGradientIterations; k++)
            {
                Vector3 Ap = GMul(p, lever);
                float denom = Mathf.Max(1e-9f, Vector3.Dot(p, Ap));
                float a = rs / denom;
                v += a * p;
                r -= a * Ap;
                float rsNew = Vector3.Dot(r, r);
                if (rsNew < 1e-8f) break;
                p = r + (rsNew / rs) * p;
                rs = rsNew;
            }

            return v;
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
