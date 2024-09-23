using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Panel : MonoBehaviour
{
    public float Timer = 0f;
    public float logInterval = 0.05f; // Time interval for logging in seconds

    // Start is called before the first frame update
    void Start()
    {
        ClearConsole();
    }

    // Update is called once per frame
    void Update()
    {
        Timer += Time.deltaTime;
        if (Timer >= logInterval)
        {
            Timer = 0f;

            // Clear the console at the start of each frame
            //ClearConsole();
        }
    }

    void ClearConsole()
    {
        #if UNITY_EDITOR
        var assembly = System.Reflection.Assembly.GetAssembly(typeof(UnityEditor.Editor));
        var type = assembly.GetType("UnityEditor.LogEntries");
        var method = type.GetMethod("Clear");
        method.Invoke(null, null);
        #endif
    }
}
