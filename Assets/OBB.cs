using UnityEngine;

public struct OBB
{

	public OBB(Transform tr)
	{
		trans = tr;
		mass = tr.GetComponent<Rigidbody>().mass;
		center = tr.position;
		extents = tr.localScale / 2.0f;
		forward = tr.forward;
		right = tr.right;
		up = tr.up;
		//float x = 1;
		//float y = 1;
		//float z = 1;
		//m_inertia.x = mass / 12.0f * (y + z);
		//m_inertia.y = mass / 12.0f * (x + z);
		//m_inertia.z = mass / 12.0f * (x + y);
		m_inertia = tr.GetComponent<Rigidbody>().inertiaTensor;
	}

	public Vector3 center;
	public Vector3 extents;
	public Vector3 right;
	public Vector3 up;
	public Vector3 forward;
	public float mass;
	public Transform trans;

	/// <summary>
	/// 转动惯量
	/// </summary>
	private Vector3 m_inertia;

	/// <summary>
	/// 求obb在世界坐标系下的惯性张量。
	/// 因为obb是对称的，各个惯性积是0，这样就得到了局部坐标系下的惯性张量，再变换到世界坐标系即可
	/// </summary>
	public Matrix4x4 InvInertiaTensor(Quaternion rotation)
	{
		// 到世界坐标的旋转变换矩阵
		Matrix4x4 mat = new Matrix4x4();
		mat.SetTRS(Vector3.zero, rotation, Vector3.one);
		Vector3 invInertia;
		invInertia.x = 1.0f / m_inertia.x;
		invInertia.y = 1.0f / m_inertia.y;
		invInertia.z = 1.0f / m_inertia.z;
		Matrix4x4 oldmat = mat;
		for (int i = 0; i < 3; i++) {
			Vector3 v = mat.GetRow(i);
			v.Scale(invInertia);
			mat.SetRow(i, v);
		}
		return mat * oldmat.transpose;
	}

	/// <summary>
	/// 空间中任意一点到obb上的最近点
	/// </summary>
	/// <returns>返回给定点到最近点的距离</returns>
	public float ClostestPntOnOBB(Vector3 origin, out Vector3 point)
	{
		bool isin = true;
		Vector3 dir = origin - center;
		Vector3 xyz = Vector3.zero;

		Vector3[] norms = {right, up, forward};

		for (int i = 0; i < 3; i++) {
			xyz[i] = Vector3.Dot(norms[i], dir);
			if (xyz[i] > extents[i] || xyz[i] < -extents[i]) {
				// 在obb外面
				isin = false;
			}
		}

		if (isin) {
			// 先找最近的面
			int mini = 0;
			float mindist = extents[0] - Mathf.Abs(xyz[0]);
			for (int i = 1; i < 3; i++) {
				float dist = extents[i] - Mathf.Abs(xyz[i]);
				if (dist < mindist) {
					mindist = dist;
					mini = i;
				}
			}
			if (xyz[mini] >= 0) {
				point = origin + norms[mini] * mindist; 
			}
			else {
				point = origin - norms[mini] * mindist;
			}
			return -mindist;
		}
		else {
			for (int i = 0; i < 3; i++) {
				xyz[i] = Mathf.Min(xyz[i], extents[i]);
				xyz[i] = Mathf.Max(xyz[i], -extents[i]);
			}
			point = center + xyz;
			return Vector3.Distance(origin, point);
		}
	}

	/// <summary>
	/// 求点在obb内部时，到obb的最近距离，以及最近面的法线。
	/// </summary>
	[System.Obsolete("在点外面时结果是错的，弃用", true)]
	public float MinDistanceWhenPntInOBB(Vector3 origin, out Vector3 normal)
	{
		normal = Vector3.zero;
		float mindst = 0.0f;
		Vector3 p = center + forward * extents.z;
		Vector3 dir = origin - p;
		float dst = Vector3.Dot(forward, dir);
		if (dst > 0) {
			// 在外面
			return 1.0f;
		}
		else {
			mindst = dst;
			normal = forward;
		}

		p = center + right * extents.x;
		dir = origin - p;
		dst = Vector3.Dot(right, dir);
		if (dst > 0) {
			// 在外面
			return 1.0f;
		}
		else if (dst > mindst) {    // dst是小于0的，越接近0越好
			mindst = dst;
			normal = right;
		}

		p = center + up * extents.y;
		dir = origin - p;
		dst = Vector3.Dot(up, dir);
		if (dst > 0) {
			// 在外面
			return 1.0f;
		}
		else if (dst > mindst) {
			mindst = dst;
			normal = up;
		}

		p = center - forward * extents.z;
		dir = origin - p;
		dst = Vector3.Dot(-forward, dir);
		if (dst > 0) {
			// 在外面
			return 1.0f;
		}
		else if (dst > mindst) {
			mindst = dst;
			normal = -forward;
		}

		p = center - right * extents.x;
		dir = origin - p;
		dst = Vector3.Dot(-right, dir);
		if (dst > 0) {
			// 在外面
			return 1.0f;
		}
		else if (dst > mindst) {
			mindst = dst;
			normal = -right;
		}

		p = center - up * extents.y;
		dir = origin - p;
		dst = Vector3.Dot(-up, dir);
		if (dst > 0) {
			// 在外面
			return 1.0f;
		}
		else if (dst > mindst) {
			mindst = dst;
			normal = -up;
		}

		return mindst;
	}

}
