using System;
using System.Collections.Generic;
using ControlHandler;
using UnityEngine;
using UnityEngine.InputSystem;
using UnityEngine.Serialization;

namespace DroneMovement
{
    public enum WingDir { CCW = 0, CW = 1 }
    public enum WingPos { Front = 0, Middle = 1, Back = 2 }
     
    [RequireComponent(typeof(PlayerInput))]
    [RequireComponent(typeof(Rigidbody))]
    public class DroneController : MonoBehaviour
    {
        internal FsSm600Handler Handler;
        internal GeneralInputHandler GeneralHandler;
        internal bool HandlerFound;
        internal float HandlerPollTime;
        internal readonly float HandlerPollTimer = 3f;
        internal SextupleAxesManager PrevAxes;
        internal SextupleAxesManager GeneralAxes;
        internal bool[] Changed = new bool[6];

        private Rigidbody _rigidBody;
        private readonly float[] _wingsZActionFactor = new float[6];
        private readonly float[] _wingsStrafeFactor = new float[6];

        [Header("Properties")] 
        [SerializeField] private GameObject model;

        [SerializeField] private Rigidbody rigidbody;
        [SerializeField] private int wingsCount = 6;
        [SerializeField] private Transform[] wings = new Transform[6];

        [Header("Controller")]
        [SerializeField] private Vector2 rcControllerDeadzone = new(.001f, .001f);
        [SerializeField] private Vector2 generalDeadzone = new(.075f, .075f);
        [SerializeField] private Vector2 rcControllerMaxValue = new(.9f, .9f);
        [SerializeField] private Vector2 generalMaxValue = new(1f, 1f);

        [Header("Movement")] 
        [SerializeField] private float throttleForce = 1f;
        [SerializeField] private float rotationForce = 10f;
        [SerializeField] private float maxTiltDegree = 30f;
        [SerializeField] private float zActionMaxRate = 1f;
        [SerializeField] private float strafeRate = 1f;
        [SerializeField] HelperPlane xzPlane;
        private float _wingRotationMultiplier = 4f;
        
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

            _rigidBody = GetComponent<Rigidbody>();
            xzPlane ??= transform.Find("XZPlane").GetComponent<HelperPlane>();

            if (wingsCount > 0 && wings[0] == null)
            {
                switch (wingsCount)
                {
                    case 6:
                        wings = new[]
                        {
                            model.transform.Find("Propeller CW F"),
                            model.transform.Find("Propeller CCW F"),
                            model.transform.Find("Propeller CW M"),
                            model.transform.Find("Propeller CCW M"),
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
            Array.Fill(_wingsStrafeFactor, 0f, 0, wings.Length);
            Array.Fill(_wingsZActionFactor, 0f, 0, wings.Length);
            Debug.Log($"{_wingsStrafeFactor.Length}, {_wingsZActionFactor.Length}");
        }

        void Update()
        {
            // for RC Controller
            if (HandlerFound)
            {
                HandlerFound = Handler.Poll();
                if (!HandlerFound) return;
                
                Handler.SendMessages(gameObject, Handler.Changes);
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
            }
            
            // execute movement
            float dT = Time.smoothDeltaTime;
            
            transform.localRotation = Quaternion.Slerp(
                transform.localRotation,
                _rotQ,
                dT
                
            );
            transform.localPosition = Vector3.Lerp(
                transform.localPosition,
                _movV,
                dT
            );
            CalculateDistanceFromWingsToXZ(wings, xzPlane.transform, _wingsStrafeFactor);
        }

        // action for the attached game object
        public void ActionLeft(Vector2 v)
        {
            _movV += new Vector3(
                0, 
                throttleForce * v.y, 
                0
            );
            _rotQ = Quaternion.Euler(
                _rotQ.eulerAngles.x,
                _rotQ.eulerAngles.y + rotationForce * v.x,
                _rotQ.eulerAngles.z
            );
        }

        public void ActionRight(Vector2 v)
        {
            _movV += transform.TransformDirection(new Vector3(
                throttleForce * v.x, 
                0,
                throttleForce * v.y
            ));
            xzPlane.Tilt(v);
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
         * CW  CCW CW  CCW CW  CCW
         * 0   0   1   1   2   2
         * 1   -1  1   -1  1   -1
         *
         * 2 * Pos + Dir
         * -1 + 2 * index % 1
         *
         * Case 4:
         * 0   1   2   3
         * F   F   B   B
         * CW  CCW CCW  CW
         * 0   0   2   2
         * 1   -1  -1  1
         */
        
        // drone animation (complex)
        private void MovementRotateLeft()
        {
            
        }
        private void MovementRotateRight()
        {
            
        }
        private void MovementY()
        {
            
        }
        private void MovementXZ()
        {
            
        }

        /*
         * Torque -> Left Thumb -> Apply Up -> Spins Clockwise
         */
        
        // drone animation (unit)
        private void RotateWing(Rigidbody body, Transform wing, WingDir d, float f)
        {
            wing.Rotate(
                0, 
                (float)d * _wingRotationMultiplier * f,
                0
            );
            body.AddRelativeTorque(
                (float)d * Vector3.up * f,
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
