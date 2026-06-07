using UnityEngine;

namespace DroneInput
{
    public struct DroneStickInput
    {
        public float roll;
        public float pitch;
        public float yaw;
        public float throttle;
    }

    public class DroneInputMapper
    {
        public enum InputMode
        {
            Keyboard,
            Gamepad
        }

        public InputMode mode = InputMode.Keyboard;

        public float rollSensitivity = 1f;
        public float pitchSensitivity = 1f;
        public float yawSensitivity = 1f;

        private float _throttleAxis = 0f;
        public float throttleSmoothTime = 0.3f;

        public DroneStickInput ReadInput(float dt)
        {
            DroneStickInput input = new DroneStickInput();

            if (mode == InputMode.Keyboard)
            {
                float roll = 0f, pitch = 0f, yaw = 0f;
                if (UnityEngine.Input.GetKey(KeyCode.A) || UnityEngine.Input.GetKey(KeyCode.LeftArrow))
                    roll = -1f;
                if (UnityEngine.Input.GetKey(KeyCode.D) || UnityEngine.Input.GetKey(KeyCode.RightArrow))
                    roll = 1f;
                if (UnityEngine.Input.GetKey(KeyCode.W) || UnityEngine.Input.GetKey(KeyCode.UpArrow))
                    pitch = -1f;
                if (UnityEngine.Input.GetKey(KeyCode.S) || UnityEngine.Input.GetKey(KeyCode.DownArrow))
                    pitch = 1f;
                if (UnityEngine.Input.GetKey(KeyCode.Q))
                    yaw = -1f;
                if (UnityEngine.Input.GetKey(KeyCode.E))
                    yaw = 1f;

                input.roll = roll * rollSensitivity;
                input.pitch = pitch * pitchSensitivity;
                input.yaw = yaw * yawSensitivity;

                float targetThrottle = 0f;
                if (UnityEngine.Input.GetKey(KeyCode.Space))
                    targetThrottle = 1f;
                if (UnityEngine.Input.GetKey(KeyCode.LeftShift))
                    targetThrottle = -0.5f;

                _throttleAxis = Mathf.MoveTowards(_throttleAxis, targetThrottle, dt / throttleSmoothTime);
                input.throttle = Mathf.Clamp01(_throttleAxis + 0.5f);
            }
            else
            {
                input.roll = UnityEngine.Input.GetAxis("LeftStickHorizontal") * rollSensitivity;
                input.pitch = -UnityEngine.Input.GetAxis("LeftStickVertical") * pitchSensitivity;
                input.yaw = UnityEngine.Input.GetAxis("RightStickHorizontal") * yawSensitivity;

                float triggerR = UnityEngine.Input.GetAxis("RightTrigger");
                float triggerL = UnityEngine.Input.GetAxis("LeftTrigger");

                float targetThrottle = triggerR;
                _throttleAxis = Mathf.MoveTowards(_throttleAxis, targetThrottle, dt / throttleSmoothTime);
                input.throttle = Mathf.Clamp01(_throttleAxis);
            }

            return input;
        }

        public void Reset()
        {
            _throttleAxis = 0f;
        }
    }
}
