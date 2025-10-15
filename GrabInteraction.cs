using System;
using UnityEngine;

public class GrabInteraction : MonoBehaviour
{
    private MeshCollider _collider;
    private XpbdMesh _xpbdMesh;
    private Vector3 _rayDir;
    private bool _mouseIsDown;
    private int _currentVertexToMove = -1;
    private float originalInvMass;

    private void Awake()
    {
        _collider = gameObject.GetComponent<MeshCollider>();
        _xpbdMesh = gameObject.GetComponent<XpbdMesh>();
    }

    private void Update()
    {
        _collider.sharedMesh = gameObject.GetComponent<MeshFilter>().sharedMesh;
        _collider.sharedMesh.RecalculateBounds();
        
        ListenForInputs();
    }

    private void ListenForInputs()
    {
        if (_mouseIsDown && _currentVertexToMove != -1)
        {
            DragVertex(_currentVertexToMove);
        }

        if (Input.GetMouseButtonDown(0) && !_mouseIsDown)
        {
            _mouseIsDown = true;
            Camera cam = Camera.main;
            Ray ray = cam.ScreenPointToRay(Input.mousePosition);
            _rayDir = ray.direction; // Das ist die richtige Welt-Richtung des Strahls

            if (RaycastNearestVertex(
                    meshCol: _collider,
                    origin: ray.origin,
                    direction: ray.direction,
                    out Vector3 nearestVtx,
                    out int idx,
                    maxDistance: 1000f,
                    exactSearch: false)) // auf true stellen für exakte Suche
            {
                _currentVertexToMove = idx;
                originalInvMass = _xpbdMesh.Particles[idx].InvMass;
            }
            else
            {
                _currentVertexToMove = -1;
            }
        }

        if (Input.GetMouseButtonUp(0))
        {
            if (_currentVertexToMove != -1)
            {
                _xpbdMesh.Particles[_currentVertexToMove].InvMass = originalInvMass;
            }
            _mouseIsDown = false;
            _currentVertexToMove = -1;
        }
    }

    public static Vector3 ProjectPointOntoRay(Vector3 point, Ray ray)
    {
        Vector3 d = ray.direction;
        float d2 = d.sqrMagnitude;
        if (d2 <= Mathf.Epsilon)
            return ray.origin; // Degenerierter Ray

        float t = Vector3.Dot(point - ray.origin, d) / d2; // Parameter entlang des (unnormierten) d
        if (t < 0f) t = 0f; // Klammern: nur Ray, keine unendliche Linie
        return ray.origin + d * t;
    }

    private void DragVertex(int id)
    {
        Ray ray = Camera.main.ScreenPointToRay(Input.mousePosition);
        Vector3 localStart = _xpbdMesh.Particles[_currentVertexToMove].Position;
        Vector3 startingPoint = _xpbdMesh.gameObject.transform.TransformPoint(localStart);
        Vector3 endPoint = _xpbdMesh.gameObject.transform.InverseTransformPoint(ProjectPointOntoRay(startingPoint, ray));

        Debug.DrawLine(startingPoint, endPoint, Color.red);
        _xpbdMesh.Particles[_currentVertexToMove] = new Particle()
        {
            Velocity = Vector3.zero,
            Position = endPoint,
            InvMass = 0
        };
    }

    public static bool RaycastNearestVertex(
        MeshCollider meshCol,
        Vector3 origin,
        Vector3 direction,
        out Vector3 nearestVertexWorld,
        out int nearestVertexIndex,
        float maxDistance = Mathf.Infinity,
        bool exactSearch = false)
    {
        nearestVertexWorld = default;
        nearestVertexIndex = -1;

        if (meshCol == null || meshCol.sharedMesh == null)
            return false;

        var ray = new Ray(origin, direction.normalized);

        // collider.Raycast testet nur gegen diesen spezifischen Collider
        if (!meshCol.Raycast(ray, out RaycastHit hit, maxDistance))
            return false;

        var mesh = meshCol.sharedMesh;
        var verts = mesh.vertices; // lokale Mesh-Vertices
        var trs = meshCol.transform;

        // Schnelle Variante: nur die 3 Vertices des getroffenen Dreiecks prüfen
        if (!exactSearch && hit.triangleIndex >= 0)
        {
            int triIndex = hit.triangleIndex * 3;
            var tris = mesh.triangles;

            int i0 = tris[triIndex];
            int i1 = tris[triIndex + 1];
            int i2 = tris[triIndex + 2];

            Vector3 v0w = trs.TransformPoint(verts[i0]);
            Vector3 v1w = trs.TransformPoint(verts[i1]);
            Vector3 v2w = trs.TransformPoint(verts[i2]);

            float d0 = (v0w - hit.point).sqrMagnitude;
            float d1 = (v1w - hit.point).sqrMagnitude;
            float d2 = (v2w - hit.point).sqrMagnitude;

            if (d0 <= d1 && d0 <= d2)
            {
                nearestVertexWorld = v0w;
                nearestVertexIndex = i0;
            }
            else if (d1 <= d0 && d1 <= d2)
            {
                nearestVertexWorld = v1w;
                nearestVertexIndex = i1;
            }
            else
            {
                nearestVertexWorld = v2w;
                nearestVertexIndex = i2;
            }

            return true;
        }

        return false;
    }

    private void OnDrawGizmos()
    {
        Gizmos.DrawRay(Camera.main.transform.position, _rayDir);
        if (_currentVertexToMove != -1)
            Gizmos.DrawSphere(_xpbdMesh.Particles[_currentVertexToMove].Position, 1f);
    }
}