using System;
using System.Collections.Generic;
using ControlHandler;
using Unity.VisualScripting;
using UnityEngine;
using UnityEngine.InputSystem;
using UnityEngine.Serialization;

namespace DroneMovement
{
    public enum WingDir { CCW = -1, CW = 1 }
    public enum WingPos { Front = 0, Middle = 1, Back = 2 }
     
    [RequireComponent(typeof(PlayerInput))]
    public class DroneController : MonoBehaviour
    {
        internal FsSm600Handler Handler;
        internal GeneralInputHandler GeneralHandler;
        internal bool HandlerFound;
        internal float HandlerPollTime;
        internal readonly float HandlerPollTimer = 3f;
        internal SextupleAxesManager PrevAxes;
        internal SextupleAxesManager GeneralAxes;

        private readonly Quaternion[] _wingRotations = new Quaternion[6];
        private readonly float[] _wingsXzDistances = new float[6];
        private readonly Quaternion[] _wingRotationsFromStrafe = new Quaternion[6];

        [Header("Properties")] 
        [SerializeField] private GameObject model;

        [SerializeField] private Rigidbody rigidBody;
        [SerializeField] private int wingsCount = 6;
        [SerializeField] private Transform[] wings = new Transform[6];

        [Header("Controller")]
        [SerializeField] private Vector2 rcControllerDeadzone = new(.001f, .001f);
        [SerializeField] private Vector2 generalDeadzone = new(.075f, .075f);
        [SerializeField] private Vector2 rcControllerMaxValue = new(.9f, .9f);
        [SerializeField] private Vector2 generalMaxValue = new(1f, 1f);

        [Header("Movement")] 
        [SerializeField] private float baseThrottleForce = .1f;
        [SerializeField] private float throttleForce = .1f;
        [SerializeField] private float rotationalMultiplier = 40f; // for movement
        [SerializeField] private float strafeRate = 1f;
        [SerializeField] HelperPlane xzPlane;
        private bool _isPowered;
        private float _currentPoweredThrottle;
        private float _wingRotationMultiplier = 16f; // for cosmetic wing rotation
        
        private Vector3 _movV;
        private Quaternion _rotQ;

        void Awake()
        {
            Handler = new FsSm600Handler();
            GeneralHandler = new GeneralInputHandler();
            HandlerFound = Handler.Device != null;
            PrevAxes = new SextupleAxesManager();
            GeneralAxes = new SextupleAxesManager();
            
            Handler.Axes.SetDeadzoneLeft(rcControllerDeadzone);
            Handler.Axes.SetDeadzoneRight(rcControllerDeadzone);
            GeneralAxes.SetDeadzoneLeft(generalDeadzone);
            GeneralAxes.SetDeadzoneRight(generalDeadzone);
            
            Handler.Axes.SetMaxValueLeft(rcControllerMaxValue);
            Handler.Axes.SetMaxValueRight(rcControllerMaxValue);
            GeneralAxes.SetMaxValueLeft(generalMaxValue);
            GeneralAxes.SetMaxValueRight(generalMaxValue);

            xzPlane ??= transform.Find("XZPlane").GetComponent<HelperPlane>();

            // TODO: ACTUALLY IMPLEMENT POWER TOGGLE
            _isPowered = true;
            
            if (wingsCount > 0 && wings[0] == null)
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

        void Update()
        {
            float dT = Time.deltaTime;
            
            // for RC Controller
            if (HandlerFound)
            {
                HandlerFound = Handler.Poll();
                if (!HandlerFound) return;
                
                Handler.SendMessages(gameObject, Handler.Changes);
            }
            else
            {
                HandlerPollTime += dT;
                if (HandlerPollTime >= HandlerPollTimer)
                {
                    HandlerPollTime = 0;
                    Handler.TryToConnect(
                        FsSm600Handler.Names, FsSm600Handler.ControlNames
                    );
                    HandlerFound = Handler.Device != null;
                }
            }
            
            // execute movement
            // -- ROTATION SECTION --
            
            transform.localRotation = Quaternion.SlerpUnclamped(
                transform.localRotation,
                _rotQ,
                dT
            );
            // -- END OF ROTATION SECTION --
            transform.localPosition = Vector3.Lerp(
                transform.localPosition,
                _movV,
                dT
            );
            _currentPoweredThrottle = Mathf.Lerp(
                _currentPoweredThrottle, 
                _isPowered ? baseThrottleForce : 0, 
                dT
            );
            
            for (int i = 0; i < wingsCount; i++)
            {
                float d = (float) WingDirAtOrder(i);
                AddWingRotation(
                    d, 
                    d, 
                    ref _wingRotations[i]
                );
            }
            CalculateDistanceFromWingsToXZ(wings, xzPlane.transform, _wingsXzDistances);
            for (int i = 0; i < wingsCount; i++)
            {
                wings[i].localRotation = Quaternion.Slerp(
                    wings[i].localRotation,
                    _wingRotationsFromStrafe[i] * _wingRotations[i],
                    dT
                );
            }
            // Debug.Log($"{wings[0].rotation},{wings[1].rotation},{wings[2].rotation}," +
            //           $"{wings[3].rotation},{wings[4].rotation},{wings[5].rotation}");
        }

        // action for the attached game object
        public void ActionLeft(Vector2 v)
        {
            //*/ apply force directly to the body
            _movV += new Vector3(
                0, 
                throttleForce * v.y, 
                0
            );
            _rotQ = Quaternion.Euler(
                _rotQ.eulerAngles.x,
                _rotQ.eulerAngles.y + throttleForce * rotationalMultiplier * v.x,
                _rotQ.eulerAngles.z
            );
            /*/
            MovementRotate(v.x);
            MovementY(v.y);
            //*/
        }

        public void ActionRight(Vector2 v)
        {
            //*/ apply force directly to the body
            xzPlane.Tilt(v);
            _movV += transform.TransformDirection(new Vector3(
                throttleForce * v.x, 
                0,
                throttleForce * v.y
            ));
            /*/
            xzPlane.Tilt(v);
            MovementXZ();
            //*/
        }

        public void ActionAux(float v)
        {
        }

        public void ActionTrigger(float v)
        {
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
                AddWingRotationAndTorque(
                    wings[i], 
                    d, 
                    d * throttleForce * v, 
                    ref _wingRotations[i], 
                    rigidBody
                );
            }
        }
        private void MovementY(float v)
        {
            for (int i = 0; i < wingsCount; i++)
            {
                float d = 1f;
                AddWingRotationAndTorque(
                    wings[i],
                    d,
                    d * throttleForce * v, 
                    ref _wingRotations[i], 
                    rigidBody
                );
            }
        }
        private void MovementXZ()
        {
            for (int i = 0; i < wingsCount; i++)
            {
                float d = (float) WingDirAtOrder(i);
                AddWingRotationAndTorque(
                    wings[i],
                    d,
                    d * throttleForce * _wingsXzDistances[i],
                    ref _wingRotationsFromStrafe[i],
                    rigidBody
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
        /// Adds rotation for a wing in `d`irection by `f`orce to its `q`uaternion variable.
        /// This is purely for cosmetic purposes.
        /// </summary>
        /// <param name="d">The direction of the rotation. 1 for CW, -1 for CCW.</param>
        /// <param name="f">Amount of the force to apply.</param>
        /// <param name="q">Variable to store the rotation of the wing.</param>
        private void AddWingRotation(
            float d,
            float f,
            ref Quaternion q
        )
        {
            q *= Quaternion.Euler(
                0, 
                d * _wingRotationMultiplier * (_currentPoweredThrottle + f),
                0
            );
        }
        /// <summary>
        /// Adds rotation for `w`ing in `d`irection by `f`orce to `q`uaternion variable,
        /// and apply torque and force to the `b`ody the wing is attached to.
        /// </summary>
        /// <param name="w">Transform of the wing to rotate.</param>
        /// <param name="d">The direction of the rotation. 1 for CW, -1 for CCW.</param>
        /// <param name="f">Amount of the force to apply.</param>
        /// <param name="q">Variable to store the rotation of the wing.</param>
        /// <param name="b">Rigidbody of the body the wing is attached to.</param>
        private void AddWingRotationAndTorque(
            Transform w,
            float d,
            float f,
            ref Quaternion q,
            Rigidbody b
        )
        {
            AddWingRotation(d, f, ref q);
            
            b.AddRelativeTorque(
                -d * Vector3.up * (_currentPoweredThrottle + f),
                ForceMode.Acceleration
            );
            b.AddForceAtPosition(
                Vector3.up * (_currentPoweredThrottle + f),
                model.transform.position + w.position,
                ForceMode.Acceleration
            );
        }

        private void CalculateDistanceFromWingsToXZ(
            IList<Transform> wingPos, Transform XZ, IList<float> dist
        )
        {
            Vector3 n = XZ.up.normalized;
            float denom = n.y;
            Vector3 p0 = XZ.position;

            for (int i = 0; i < wingPos.Count; i++)
            {
                Vector3 wp = wingPos[i].position;
                dist[i] = (Mathf.Abs(denom) < 1e-6f)
                    ? float.PositiveInfinity
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
        public void OnLeft(InputValue v) => 
            ActionLeft(GeneralHandler.OnLeft(v));
        public void OnRight(InputValue v) => 
            ActionRight(GeneralHandler.OnRight(v));
        public void OnAux(InputValue v) => 
            ActionAux(GeneralHandler.OnAux(v));
        public void OnTrigger(InputValue v) => 
            ActionTrigger(GeneralHandler.OnTrigger(v));

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
