using UnityEngine;

public struct OBB
{

	public OBB(Transform tr)
	{
		mass = tr.GetComponent<Rigidbody>().mass;
		center = tr.position;
		extents = Vector3.one / 2.0f;
		forward = tr.forward;
		right = tr.right;
		up = tr.up;
		float x = extents.x * extents.x;
		float y = extents.y * extents.y;
		float z = extents.z * extents.z;
		m_inertia.x = mass / 12.0f * (y + z);
		m_inertia.y = mass / 12.0f * (x + z);
		m_inertia.z = mass / 12.0f * (x + y);
		extents.Scale(tr.localScale);
	}

	public Vector3 center;
	public Vector3 extents;
	public Vector3 right;
	public Vector3 up;
	public Vector3 forward;
	public float mass;

	/// <summary>
	/// 转动惯量
	/// </summary>
	private Vector3 m_inertia;

	/// <summary>
	/// 求obb的惯性张量。bullet就是这么干的，我也不知道为啥
	/// </summary>
	public Matrix4x4 InertiaTensor(Quaternion rotation)
	{
		// 旋转矩阵
		Matrix4x4 mat = new Matrix4x4();
		mat.SetTRS(Vector3.zero, rotation, Vector3.one);
		mat.GetColumn(0).Scale(m_inertia);
		mat.GetColumn(1).Scale(m_inertia);
		mat.GetColumn(2).Scale(m_inertia);
		return mat * mat.transpose;
	}

	/// <summary>
	/// 求点在obb内部时，到obb的最近距离，以及最近面的法线。在点外面时结果是错的
	/// </summary>
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
		else if (dst > mindst) {	// dst是小于0的，越接近0越好
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
