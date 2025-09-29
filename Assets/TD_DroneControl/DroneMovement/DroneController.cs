using System.Collections.Generic;
using ControlHandler;
using UnityEngine;
using UnityEngine.InputSystem;
using UnityEngine.Serialization;

namespace DroneMovement
{
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
        private Transform[] _wings;
        private float[] _wingsZActionFactor;
        private float[] _wingsStrafeFactor;

        [Header("Controller")]
        [SerializeField] private float rcControllerDeadzone = .001f;
        [SerializeField] private float generalDeadzone = .075f;
        [SerializeField] private float rcControllerMaxValue = .9f;
        [SerializeField] private float generalMaxValue = 1f;

        [Header("Movement")] 
        [SerializeField] private float throttleForce = 2f;
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
            
            Handler.Axes.SetMaxValueLeft(rcControllerMaxValue * Vector2.one);
            Handler.Axes.SetMaxValueRight(rcControllerMaxValue * Vector2.one);
            GeneralAxes.SetMaxValueLeft(generalMaxValue * Vector2.one);
            GeneralAxes.SetMaxValueRight(generalMaxValue * Vector2.one);

            _rigidBody = GetComponent<Rigidbody>();
            xzPlane ??= transform.Find("XZPlane").GetComponent<HelperPlane>();
            _wings = new[]
            {
                transform.Find("Propeller CW F"),
                transform.Find("Propeller CCW F"),
                transform.Find("Propeller CW M"),
                transform.Find("Propeller CCW M"),
                transform.Find("Propeller CW B"),
                transform.Find("Propeller CCW B"),
            };
            _wingsStrafeFactor = new[]
            {
                0f, 0f, 0f, 0f, 0f, 0f
            };
            _wingsZActionFactor = new[]
            {
                0f, 0f, 0f, 0f, 0f, 0f
            };
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

        // drone animation
        private void RotateWings()
        {
        }

        private void CalculateDistanceFromWingsToXZ(
            List<Transform> wingPos, Transform XZ, IList<float> dist
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
