// OrbitCamera.cs
// Simple scene orbit/pan/zoom camera suited for inspecting a spherical globe.
// Put this on your Main Camera and assign Target to your globe root.

using UnityEngine;

[ExecuteAlways]
[DisallowMultipleComponent]
public class OrbitCamera : MonoBehaviour
{
    [Header("Target")]
    public Transform Target;
    [Tooltip("Approx radius of the thing you're viewing; used for initial distance & Focus.")]
    public float TargetRadius = 1.0f;

    [Header("Distance")]
    public float Distance = 3.0f;
    public float MinDistance = 0.5f;
    public float MaxDistance = 50.0f;
    [Tooltip("Mouse wheel zoom speed multiplier.")]
    public float ZoomSpeed = 5.0f;
    [Tooltip("Exponential damping for zoom (0=no damping, 20=snappy).")]
    [Range(0, 30)] public float ZoomDamping = 12f;

    [Header("Orbit")]
    [Tooltip("Degrees per pixel horizontally/vertically.")]
    public float OrbitXSensitivity = 10f;
    public float OrbitYSensitivity = 10f;
    [Tooltip("Clamp pitch to avoid flipping.")]
    public float MinPitch = -85f;
    public float MaxPitch = 85f;
    [Tooltip("Exponential damping for orbit (0=no damping).")]
    [Range(0, 30)] public float OrbitDamping = 12f;

    [Header("Pan")]
    [Tooltip("World-units panned per pixel at Distance=1. Scales with Distance.")]
    public float PanSensitivity = 0.0025f;
    [Tooltip("Exponential damping for pan (0=no damping).")]
    [Range(0, 30)] public float PanDamping = 12f;

    [Header("Keys")]
    public KeyCode FocusKey = KeyCode.F;

    // State
    private float _yaw, _pitch;
    private float _targetYaw, _targetPitch;
    private float _targetDistance;
    private Vector3 _targetPivot;   // world-space pivot (usually Target.position)
    private Vector3 _pivot;         // smoothed

    void Reset()
    {
        if (!Target) TryAutoFindTarget();
        if (Target) _targetPivot = _pivot = Target.position;
        if (TargetRadius <= 0f) TargetRadius = 1f;
        Distance = _targetDistance = Mathf.Max(TargetRadius * 2.5f, 3f);
        InitAnglesFromCurrentTransform();
    }

    void OnEnable()
    {
        if (!Target) TryAutoFindTarget();
        if (Target) _targetPivot = _pivot = Target.position;
        if (_targetDistance <= 0f) _targetDistance = Distance > 0 ? Distance : Mathf.Max(TargetRadius * 2.5f, 3f);
        if (Distance <= 0f) Distance = _targetDistance;
        InitAnglesFromCurrentTransform();
        ApplyTransformInstant();
    }

    void Update()
    {
        if (!Target)
        {
            TryAutoFindTarget();
            if (!Target) return;
            _targetPivot = _pivot = Target.position;
        }

        // Keep pivot on target (you can pan away from it; we store pan in _pivot directly).
        if (Target) _targetPivot += (Target.position - _targetPivot);

        HandleInput();
        SmoothTowardsTargets(Time.unscaledDeltaTime);
        ApplyTransform();
    }

    private void HandleInput()
    {
        // Orbit: RMB drag or Alt+LMB (like Scene View)
        bool orbit = Input.GetMouseButton(1) || (Input.GetMouseButton(0) && (Input.GetKey(KeyCode.LeftAlt) || Input.GetKey(KeyCode.RightAlt)));
        // Pan: MMB drag or Shift+RMB
        bool pan = Input.GetMouseButton(2) || (Input.GetMouseButton(1) && (Input.GetKey(KeyCode.LeftShift) || Input.GetKey(KeyCode.RightShift)));

        Vector2 mouseDelta = new Vector2(Input.GetAxis("Mouse X"), Input.GetAxis("Mouse Y"));

        if (orbit && !pan)
        {
            _targetYaw += mouseDelta.x * OrbitXSensitivity;
            _targetPitch -= mouseDelta.y * OrbitYSensitivity;
            _targetPitch = Mathf.Clamp(_targetPitch, MinPitch, MaxPitch);
        }

        if (pan)
        {
            // Pan in camera-right and camera-up directions; scale by distance so it feels consistent
            float scale = PanSensitivity * Mathf.Max(0.1f, Distance);
            Vector3 right = transform.right;
            Vector3 up = transform.up;
            _targetPivot -= (right * mouseDelta.x + up * mouseDelta.y) * scale;
        }

        // Zoom via scroll
        float scroll = Input.GetAxis("Mouse ScrollWheel");
        if (Mathf.Abs(scroll) > Mathf.Epsilon)
        {
            float speed = ZoomSpeed * (Input.GetKey(KeyCode.LeftShift) || Input.GetKey(KeyCode.RightShift) ? 2f : 1f);
            _targetDistance *= Mathf.Exp(-scroll * speed); // multiplicative zoom feels nice
            _targetDistance = Mathf.Clamp(_targetDistance, MinDistance, MaxDistance);
        }

        // Focus key
        if (Input.GetKeyDown(FocusKey))
        {
            Focus();
        }
    }

    public void Focus()
    {
        if (!Target) return;
        // Frame the target assuming roughly spherical bounds
        var r = Mathf.Max(0.001f, TargetRadius);
        _targetPivot = Target.position;
        // Place camera so that the globe roughly fills the view (depends on FOV). A simple heuristic:
        float vfov = Camera.main ? Camera.main.fieldOfView * Mathf.Deg2Rad : 60f * Mathf.Deg2Rad;
        float fitK = 1.3f; // a bit of margin
        float d = r * fitK / Mathf.Tan(vfov * 0.5f);
        d = Mathf.Clamp(d, MinDistance, MaxDistance);
        _targetDistance = d;

        // Keep current yaw/pitch; just move distance/pivot
    }

    private void SmoothTowardsTargets(float dt)
    {
        float expOrbit = OrbitDamping > 0 ? 1f - Mathf.Exp(-OrbitDamping * dt) : 1f;
        float expZoom = ZoomDamping > 0 ? 1f - Mathf.Exp(-ZoomDamping * dt) : 1f;
        float expPan = PanDamping > 0 ? 1f - Mathf.Exp(-PanDamping * dt) : 1f;

        _yaw = Mathf.LerpAngle(_yaw, _targetYaw, expOrbit);
        _pitch = Mathf.LerpAngle(_pitch, _targetPitch, expOrbit);
        Distance = Mathf.Lerp(Distance, _targetDistance, expZoom);
        _pivot = Vector3.Lerp(_pivot, _targetPivot, expPan);
    }

    private void ApplyTransform()
    {
        Quaternion rot = Quaternion.Euler(_pitch, _yaw, 0f);
        Vector3 offset = rot * (Vector3.back * Distance);
        transform.position = _pivot + offset;
        transform.rotation = rot;
    }

    private void ApplyTransformInstant()
    {
        _yaw = _targetYaw;
        _pitch = _targetPitch;
        Distance = _targetDistance;
        _pivot = _targetPivot;
        ApplyTransform();
    }

    private void InitAnglesFromCurrentTransform()
    {
        Vector3 fwd = transform.forward;
        // Yaw from forward projected onto XZ
        Vector3 fwdXZ = new Vector3(fwd.x, 0f, fwd.z).normalized;
        if (fwdXZ.sqrMagnitude < 1e-6f) _yaw = _targetYaw = 0f;
        else _yaw = _targetYaw = Mathf.Atan2(fwdXZ.x, fwdXZ.z) * Mathf.Rad2Deg;

        // Pitch from forward vs world forward in camera's yaw plane
        _pitch = _targetPitch = Mathf.Asin(Mathf.Clamp(fwd.y, -1f, 1f)) * Mathf.Rad2Deg;

        if (Target)
        {
            // Initialize distance from current transform to target
            float d = Vector3.Distance(transform.position, Target.position);
            _targetDistance = Distance = Mathf.Clamp(d, MinDistance, MaxDistance);
            _targetPivot = _pivot = Target.position;
        }
        else
        {
            if (Distance <= 0f) Distance = _targetDistance = 3f;
            _targetPivot = _pivot = Vector3.zero;
        }
    }

    private void TryAutoFindTarget()
    {
        // Try find a generator by name
        var gen = GameObject.Find("GeodesicDualSphere");
        if (gen) Target = gen.transform;
        else
        {
            // Otherwise just keep current target null; user can assign in Inspector
        }
    }

    // Draw a simple gizmo for the pivot
    void OnDrawGizmosSelected()
    {
        Gizmos.color = new Color(1f, 1f, 0f, 0.4f);
        Gizmos.DrawWireSphere(_pivot, Mathf.Max(0.01f, TargetRadius));
    }
}
