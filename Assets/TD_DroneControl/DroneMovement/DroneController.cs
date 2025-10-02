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
        internal bool TriggerToggle; // from OnTrigger
        internal float KnobMix; // from OnAux

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
        [SerializeField] private float baseThrottleForce = 5f;
        [SerializeField] private float throttleForce = 5f;
        [SerializeField] private float velocityResponse = 6f;
        [SerializeField] private float angularResponse = 6f;
        [SerializeField] private float rotationalMultiplier = 20f; // for movement
        [SerializeField] private float strafeRate = 1f;
        [SerializeField] HelperPlane xzPlane;
        [SerializeField] private float wingRotationMultiplier = 16f; // for cosmetic wing rotation
        [SerializeField] private bool isPowered;
        private float _currentPoweredThrottle;

        private Vector3 _movV = Vector3.zero;
        private Vector3 _movVSmoothed = Vector3.zero;
        private Vector3 _currentUp;
        private Quaternion _rotQ = Quaternion.identity;
        private Quaternion _rotQSmoothed = Quaternion.identity;

        void Awake()
        {
            Handler = new FsSm600Handler();
            GeneralHandler = new GeneralInputHandler();
            HandlerFound = Handler.Device != null;
            PrevAxes = new SextupleAxesManager();
            GeneralAxes = new SextupleAxesManager();
            
            UpdateAxisRange();

            xzPlane ??= transform.Find("XZPlane").GetComponent<HelperPlane>();

            // TODO: ACTUALLY IMPLEMENT POWER TOGGLE
            isPowered = true;
            
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
            
            // update powered state
            _currentPoweredThrottle = Mathf.Lerp(
                _currentPoweredThrottle, 
                isPowered ? baseThrottleForce : 0, 
                fdt
            );

            
            /* directly control the body
            // compute body movement
            _movVSmoothed = Vector3.Lerp(
                _movVSmoothed,
                transform.right * _movV.x +
                Vector3.up * _movV.y +
                transform.forward * _movV.z,
                velocityResponse * fdt
            );
            // apply body movement
            //#1#
            rigidBody.MovePosition(
                rigidBody.position + _movVSmoothed * fdt
            );
            /#1#
            rigidBody.linearVelocity = _movVSmoothed;
            //#1#

            // compute body rotation
            _rotQSmoothed = Quaternion.Slerp(
                _rotQSmoothed,
                _rotQ,
                angularResponse * fdt
            );
            Quaternion deltaRotation = Quaternion.Slerp(
                Quaternion.identity,
                _rotQSmoothed,
                fdt
            );
            // apply body rotation
            rigidBody.MoveRotation(Quaternion.Normalize(
                rigidBody.rotation * deltaRotation
            ));
            */
            
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
                    isPowered ? _wingRotationsFromStrafe[i] * _wingRotations[i] : Quaternion.identity,
                    _wingsXzDistances[i] * fdt
                );

                wings[i].Rotate(
                    Quaternion
                        .Normalize(wings[i].localRotation * deltaWingRotation)
                        .eulerAngles,
                    Space.Self
                );
            }
            /*/
            Debug.Log($"{_wingRotationsFromStrafe[0]*_wingRotations[0]}, {_wingRotationsFromStrafe[1]*_wingRotations[1]}, {_wingRotationsFromStrafe[2]*_wingRotations[2]}, {_wingRotationsFromStrafe[3]*_wingRotations[3]}, {_wingRotationsFromStrafe[4]*_wingRotations[4]}, {_wingRotationsFromStrafe[5]*_wingRotations[5]}");
            /*/
            Debug.Log($"{wings[0].localRotation},{wings[1].localRotation},{wings[2].localRotation},{wings[3].localRotation},{wings[4].localRotation},{wings[5].localRotation}");
            //*/
        }

        // action for the attached game object
        public void ActionLeft(Vector2 v)
        {
            /*/ apply force directly to the body
            _movV.y = throttleForce * v.y; // target local velocity
            _rotQ = Quaternion.Euler(
                0,
                throttleForce * rotationalMultiplier * v.x,
                0
            ); // target angular velocity
            /*/
            MovementRotate(v.x);
            MovementY(v.y);
            //*/
        }

        public void ActionRight(Vector2 v)
        {
            //*/ apply force directly to the body
            xzPlane.Tilt(v);
            _movV.x = throttleForce * v.x;
            _movV.z = throttleForce * v.y;
            
            //apply banking motion
            _rotQ = Quaternion.Euler(
                rotationalMultiplier * -v.y,
                _rotQ.eulerAngles.y,
                rotationalMultiplier * -v.x
            );
            /*/
            xzPlane.Tilt(v);
            MovementXZ();
            //*/
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
                    d * throttleForce * v, 
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
                    d * throttleForce * v, 
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
                ChangeWingRotationAndAddTorque(
                    wings[i],
                    d,
                    d * throttleForce * _wingsXzDistances[i],
                    out _wingRotationsFromStrafe[i],
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
                d * wingRotationMultiplier * (_currentPoweredThrottle + f),
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
