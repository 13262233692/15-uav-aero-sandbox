using UnityEngine;
using UnityEditor;
using FlightPhysics;

[CustomEditor(typeof(QuadcopterSimulator))]
public class QuadcopterSimulatorEditor : Editor
{
    public override void OnInspectorGUI()
    {
        DrawDefaultInspector();

        QuadcopterSimulator sim = (QuadcopterSimulator)target;

        EditorGUILayout.Space(10);
        EditorGUILayout.LabelField("Live Status", EditorStyles.boldLabel);

        EditorGUI.BeginDisabledGroup(true);
        EditorGUILayout.Vector3Field("Position", sim.CurrentPosition);
        EditorGUILayout.Vector3Field("Velocity", sim.CurrentVelocity);
        EditorGUILayout.Vector3Field("Angular Velocity", sim.CurrentAngularVelocity);
        EditorGUILayout.Toggle("Armed", sim.IsArmed);

        float[] rpms = sim.MotorRpms;
        EditorGUILayout.LabelField($"Motor RPMs: [{rpms[0]:F0}, {rpms[1]:F0}, {rpms[2]:F0}, {rpms[3]:F0}]");

        float[] pwms = sim.MotorPwms;
        EditorGUILayout.LabelField($"Motor PWMs: [{pwms[0]:F0}, {pwms[1]:F0}, {pwms[2]:F0}, {pwms[3]:F0}]");

        EditorGUILayout.Toggle("In Ground Effect", sim.InGroundEffect);
        if (sim.InGroundEffect)
        {
            GroundEffectResult[] ge = sim.GroundEffectResults;
            if (ge != null && ge.Length >= 4)
            {
                EditorGUILayout.LabelField($"GE Dist: [{ge[0].groundDistance:F2}, {ge[1].groundDistance:F2}, {ge[2].groundDistance:F2}, {ge[3].groundDistance:F2}]m");
                EditorGUILayout.LabelField($"GE Mult: [{ge[0].thrustMultiplier:F3}, {ge[1].thrustMultiplier:F3}, {ge[2].thrustMultiplier:F3}, {ge[3].thrustMultiplier:F3}]");
            }
        }
        EditorGUI.EndDisabledGroup();

        EditorGUILayout.Space(5);
        if (GUILayout.Button("Arm"))
            sim.SendMessage("Arm");
        if (GUILayout.Button("Disarm"))
            sim.SendMessage("Disarm");
        if (GUILayout.Button("Reset Position"))
        {
            Undo.RecordObject(sim.transform, "Reset Position");
            sim.transform.position = Vector3.up * 2f;
            sim.transform.rotation = Quaternion.identity;
        }

        if (GUI.changed)
            EditorUtility.SetDirty(target);
    }
}
