using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Test : MonoBehaviour
{

	public Transform tr1, tr2;

	// Use this for initialization
	void Start()
	{

	}

	// Update is called once per frame
	void Update()
	{
		int layerMask = 1 << LayerMask.NameToLayer("Terrain");
		RaycastHit hit;
		if (Physics.Linecast(tr1.position, tr2.position, out hit, layerMask)) {
			Debug.DrawLine(tr1.position, hit.point, Color.blue);
			Debug.DrawRay(hit.point, hit.normal * 10, Color.red);
		}
	}
}
