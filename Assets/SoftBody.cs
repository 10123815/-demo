using System.Collections.Generic;
using UnityEngine;

public class SoftBody : MonoBehaviour
{

	public bool isTorus = true;

	public float W = 0.1f;
	public float Stiffness = 1;
	public float Bend = 1;
	public float Friction = 0.1f;
	public float Hardness = 0.1f;
	public int INTERATIONS = 20;

	protected List<Node> m_nodes;
	protected List<Link> m_links;
	protected List<Face> m_faces;

	// 和刚体的碰撞信息
	private List<RigidContactInfo> m_rigidContactInfos = new List<RigidContactInfo>();

	// 这个相当于overlapping pairs
	public List<Collider> m_colliders = new List<Collider>();

	protected Mesh m_mesh;
	private Terrain m_terrain;

	protected class RigidContactInfo
	{
		public Matrix4x4 impMat;
		public Vector3 normal = Vector3.up;
		public Vector3 closestPnt;
		public float friction = 0.1f;
		public float hardness = 0.1f;

		public Collider collider;
		public Node node;

		public float param0;	// dt / mass
		public Vector3 param1;	// relative anchor
	}

	protected class Node
	{
		public Vector3 curpos;
		public Vector3 prevpos;
		public Vector3 velocity;
		public float w;         // 1 / mass
		public int idx;
		public object data;
	}

	protected class Link
	{
		public Vector3 dir; // 方向，n2指向n1  
		public Node n1, n2;
		public float param0; // k / (w1 + w2)
		public float param1; // 弹簧的原始长度的平方
	}

	protected class Face
	{
		public Node n1, n2, n3;
		public float area;      // 三角形面积
	}

	void Awake()
	{
		CreateSoftBody();

		m_terrain = GameObject.FindGameObjectWithTag("Terrain").GetComponent<Terrain>();

		GameObject[] boxs = GameObject.FindGameObjectsWithTag("box");
		for (int i = 0; i < boxs.Length; i++) {
			m_colliders.Add(boxs[i].GetComponent<Collider>());
		}
	}

	virtual protected void CreateSoftBody()
	{
		if (isTorus) {
			m_mesh = GetComponent<MeshFilter>().mesh = new Mesh();
			CreateSoftBodyFromMesh(TorusMeshData.gVertices, TorusMeshData.gIndices);
		}
		else {
			m_mesh = GetComponent<MeshFilter>().mesh;
			CreateSoftBodyFromMesh(m_mesh.vertices, m_mesh.triangles);
		}
	}

	// Use this for initialization
	void Start()
	{

	}

	// Update is called once per frame
	void Update()
	{
		//for (int i = 0; i < m_nodes.Count; i++) {
		//	Node node = m_nodes[i];
		//	Debug.DrawLine(node.prevpos, node.curpos, Color.blue);
		//}
	}

	void FixedUpdate()
	{
		PredictMotion();

		NarrowPhase();

		SolveContraints();

		if (m_mesh) {
			Vector3[] vertices = m_mesh.vertices;
			for (int i = 0; i < m_nodes.Count; i++) {
				Node node = m_nodes[i];
				Quaternion rotation = transform.rotation;
				rotation.w = -rotation.w;
				vertices[node.idx] = rotation * (node.curpos - transform.position);
			}
			m_mesh.vertices = vertices;
			m_mesh.RecalculateNormals();
		}
	}

	private void CreateSoftBodyFromMesh(Vector3[] vertices, int[] tris)
	{
		m_nodes = new List<Node>();
		m_links = new List<Link>();
		m_faces = new List<Face>();

		if (isTorus) {
			m_mesh.vertices = new Vector3[vertices.Length];
			m_mesh.uv = new Vector2[vertices.Length];
			m_mesh.triangles = tris;
		}

		for (int i = 0; i < vertices.Length; i++) {
			Node node = new Node();
			node.curpos = transform.rotation * vertices[i] + transform.position;
			node.prevpos = node.curpos;
			node.velocity = Vector3.zero;
			node.w = W;
			node.idx = i;
			m_nodes.Add(node);

			if (isTorus) {
				m_mesh.uv[i] = new Vector2(0, 0);
				m_mesh.vertices[i] = vertices[i];
			}
		}

		// 这个保存某一条边相对的那个点的索引，可用于计算弯曲弹簧
		// 也可以视为邻接矩阵，-1表示两个点没有弹簧相连，>=0表示连了拉伸弹簧，-2表示连了弯曲弹簧。
		// 假设两个点可以同时连拉伸和弯曲弹簧，可能不太科学
		List<int> adjacentPoints = new List<int>();
		// 最多有三角形个数*3条边，相当于顶点索引个数（所有三角形都是独立的，没有重合的边）
		for (int i = 0; i < vertices.Length * vertices.Length; i++) {
			adjacentPoints.Add(-1);
		}

		// 这里遍历全部三角形
		for (int i = 0; i < tris.Length; i += 3) {
			for (int j = 0, k = 1; j < 3; j++, k++, k %= 3) {
				int idx1 = tris[i + j];
				int idx2 = tris[i + k];
				int idx3 = tris[i + (k + 1) % 3]; // 这个就是边相对的点的索引
				if (adjacentPoints[idx1 * vertices.Length + idx2] < 0 && adjacentPoints[idx2 * vertices.Length + idx1] < 0) {
					adjacentPoints[idx1 * vertices.Length + idx2] = idx3;
					adjacentPoints[idx2 * vertices.Length + idx1] = idx3;
					// 这个弹簧用于拉伸约束
					Link link = new Link();
					link.n1 = m_nodes[idx1];
					link.n2 = m_nodes[idx2];
					float w = link.n1.w + link.n2.w;
					link.param0 = w / Stiffness;
					link.param1 = (link.n1.curpos - link.n2.curpos).sqrMagnitude;
					m_links.Add(link);
				}
				else {
					// 如果某条边属于两个三角形，则会遍历两次；如果某条边只属于一个三角形，则只会遍历一次。
					// 如果已经把一条边添加到弹簧的list里，说明是第二次遍历，这样就确定了边所属的两个三角形的邻接关系
					int idx4 = adjacentPoints[idx1 * vertices.Length + idx2];
					if (idx4 != -1) {
						// 说明idx1,idx2已经对应了一个点了
						if (adjacentPoints[idx3 * vertices.Length + idx4] != -2 && adjacentPoints[idx4 * vertices.Length + idx3] != -2) {
							adjacentPoints[idx3 * vertices.Length + idx4] = -2;
							adjacentPoints[idx4 * vertices.Length + idx3] = -2;
							// 这个弹簧用于弯曲约束
							Link link = new Link();
							link.n1 = m_nodes[idx3];
							link.n2 = m_nodes[idx4];
							float w = link.n1.w + link.n2.w;
							link.param0 = w / Bend;
							link.param1 = (link.n1.curpos - link.n2.curpos).sqrMagnitude;
							m_links.Add(link); 
						}
					}
				}
			}
			Face face = new Face();
			face.n1 = m_nodes[tris[i]];
			face.n2 = m_nodes[tris[i + 1]];
			face.n3 = m_nodes[tris[i + 2]];
			Vector3 a = face.n2.curpos - face.n1.curpos;
			Vector3 b = face.n3.curpos - face.n1.curpos;
			face.area = Vector3.Cross(a, b).magnitude / 2;
			m_faces.Add(face);
		}
	}

	protected void PredictMotion()
	{
		for (int i = 0; i < m_nodes.Count; i++) {
			if (m_nodes[i].w <= 0) {
				continue;
			}
			// 重力
			m_nodes[i].velocity += Physics.gravity * Time.fixedDeltaTime;
			m_nodes[i].prevpos = m_nodes[i].curpos;
			m_nodes[i].curpos += m_nodes[i].velocity * Time.fixedDeltaTime;
			// TODO: 其他的外力
		}

		// 假的
		transform.position = m_nodes[0].curpos;
	}

	protected void NarrowPhase()
	{
		NarrowPhaseWithOBB();
		NarrowPhaseWithTerrain();
	}

	private void NarrowPhaseWithOBB()
	{
		m_rigidContactInfos.Clear();
		for (int i = 0; i < m_colliders.Count; i++) {
			Collider cld = m_colliders[i];
			Rigidbody rigid = cld.attachedRigidbody;
			if (rigid == null) {
				continue;
			}
			Transform tr = cld.transform;
			OBB obb = new OBB(tr);
			for (int j = 0; j < m_nodes.Count; j++) {
				Node node = m_nodes[j];
				Vector3 point;
				// float dst = obb.MinDistanceWhenPntInOBB(node.curpos, out normal);
				float dst = obb.ClostestPntOnOBB(node.curpos, out point);
				if (dst <= 0) {
					Vector3 normal = (point - node.curpos).normalized;
					Vector3 ra = node.curpos - tr.position;
					Vector3 va = rigid.velocity + Vector3.Cross(rigid.angularVelocity, ra);
					va *= Time.fixedDeltaTime;
					Vector3 vb = node.curpos - node.prevpos;
					Vector3 vr = vb - va;
					// dn相当于相对速度的法相分量
					float dn = Vector3.Dot(vr, normal);
					// fv相当于相对速度的切线分量
					Vector3 fv = vr - normal * dn;	

					RigidContactInfo rci = new RigidContactInfo();
					rci.hardness = Hardness;
					rci.friction = fv.sqrMagnitude < (dn * Friction * dn * Friction) ? 0 : 1 - Friction;
					rci.normal = normal;
					rci.closestPnt = point;
					rci.node = node;
					rci.collider = cld;
					rci.param0 = node.w * Time.fixedDeltaTime;
					rci.param1 = ra;
					rci.impMat = ImpulseMatrix(node.w, 1.0f / obb.mass, obb.InertiaTensor(tr.rotation), ra);
					m_rigidContactInfos.Add(rci);
				}
			}
		}
	}

	private void NarrowPhaseWithTerrain()
	{
		for (int i = 0; i < m_nodes.Count; i++) {
			// 检测各个顶点是否与collider碰撞。不精确，有时三个点不在collider内，但是三角形却与collider相交
			Node node = m_nodes[i];
			RaycastHit hit;
			int layerMask = 1 << LayerMask.NameToLayer("Terrain");
			RigidContactInfo rci = new RigidContactInfo();

			// 检测一下质点是否在物体内，因为上一帧的impulse不一定能把质点移出到物体外，这时射线是不能喝物体相交的
			float y = m_terrain.SampleHeight(node.curpos);
			if (node.curpos.y < y) {
				if (node.prevpos.y < y) {
					// 上一帧的impulse没把质点移出物体外

					// 这样算距离太暴力了，应该算当前位置和网格的最近距离，但是TerrainCollider不支持Physics.ClosetPoint
					// float dst = node.curpos.y - y;

					// 可以通过梯度下降法求一个近似的最近点。限制最多迭代次数
					Vector3 closetPoint;
					float dst = -GradientDescent(m_terrain, node.curpos, out closetPoint);
					if (Physics.Raycast(node.curpos, closetPoint - node.curpos, out hit, layerMask)) {
						rci.normal = hit.normal;
					}
					rci.node = node;
					rci.closestPnt = closetPoint;
					rci.collider = null;
					rci.param0 = node.w * Time.fixedDeltaTime;
					rci.impMat = ImpulseMatrix(node.w, 0, Matrix4x4.zero, node.curpos);
					m_rigidContactInfos.Add(rci);
				}
				else if (Physics.Linecast(node.prevpos, node.curpos, out hit, layerMask)) {
					// 外→内
					float dst = -(node.curpos - hit.point).magnitude;
					rci.normal = hit.normal.normalized;
					rci.node = node;
					rci.collider = null;
					rci.param0 = node.w * Time.fixedDeltaTime;
					rci.closestPnt = node.curpos - rci.normal * dst;
					rci.impMat = ImpulseMatrix(node.w, 0, Matrix4x4.zero, node.curpos);
					m_rigidContactInfos.Add(rci);
				}
			}
			else if (node.prevpos.y >= y && Physics.Linecast(node.prevpos, node.curpos, out hit, layerMask)) {
				// 都在物体外，但射线穿过物体，小物体、尖锐的地方可能会出现这种情况
				float dst = -(node.curpos - hit.point).magnitude;
				rci.normal = hit.normal.normalized;
				rci.node = node;
				rci.collider = null;
				rci.param0 = node.w * Time.fixedDeltaTime;
				rci.closestPnt = node.curpos - rci.normal * dst;
				rci.impMat = ImpulseMatrix(node.w, 0, Matrix4x4.zero, node.curpos);
				m_rigidContactInfos.Add(rci);
			}

			// 内→外不算，因为线性约束已经把质点拉出物体了
		}

	}

	protected void SolveContraints()
	{
		// solve position constraints
		for (int i = 0; i < INTERATIONS; i++) {
			// contacts solver
			for (int j = 0; j < m_rigidContactInfos.Count; j++) {
				RigidContactInfo rci = m_rigidContactInfos[j];
				Node node = rci.node;
				Collider collider = rci.collider;
				Rigidbody rigid = null;

				Vector3 va = Vector3.zero;
				if (collider != null) {
					// 刚体的速度
					rigid = collider.attachedRigidbody;
					if (rigid != null) {
						va = rigid.velocity + Vector3.Cross(rigid.angularVelocity, rci.param1);
						va *= Time.fixedDeltaTime;
					}
				}
				Vector3 vb = node.curpos - node.prevpos;
				Vector3 vr = vb - va;
				float dn = Vector3.Dot(vr, rci.normal);
				if (dn <= 2.2204460492503131e-016) {
					float dp = Mathf.Min(-Vector3.Distance(rci.closestPnt, node.curpos), 0);
					Vector3 fv = vr - rci.normal * dn;
					Vector3 impulse = rci.impMat * (vr - (fv * rci.friction) + (rci.normal * dp * rci.hardness));
					node.curpos -= impulse * rci.param0;
					if (rigid != null) {
						// 刚体收到的冲量
						rigid.AddForceAtPosition(impulse, node.curpos, ForceMode.Impulse);
					} 
				}
			}
			// linear soler
			for (int j = 0; j < m_links.Count; j++) {
				Link link = m_links[j];
				if (link.param0 > 0) {
					Node n1 = link.n1;
					Node n2 = link.n2;
					Vector3 dir = n1.curpos - n2.curpos;
					float len = dir.sqrMagnitude;
					if (len + link.param1 > 1.192092896e-07F) {
						Vector3 dp = (len - link.param1) / (len + link.param1) / link.param0 * dir;
						n1.curpos -= n1.w * dp;
						n2.curpos += n2.w * dp;
					}
				}
			}
		}
		// update velocity
		for (int i = 0; i < m_nodes.Count; i++) {
			Node node = m_nodes[i];
			node.velocity = (node.curpos - node.prevpos) / Time.fixedDeltaTime;
		}

	}

	private Matrix4x4 ImpulseMatrix(float w1, float w2, Matrix4x4 iwi, Vector3 r)
	{
		Matrix4x4 mat = MassMatrix(w2, iwi, r);
		mat.SetRow(0, new Vector4(w1, 0, 0, 0) + mat.GetRow(0));
		mat.SetRow(1, new Vector4(0, w1, 0, 0) + mat.GetRow(1));
		mat.SetRow(2, new Vector4(0, 0, w1, 0) + mat.GetRow(2));
		mat.SetRow(3, new Vector4(0, 0, 0, w1) + mat.GetRow(3));
		return Diagonal(1.0f / Time.fixedDeltaTime) * mat.inverse;
	}

	private Matrix4x4 MassMatrix(float w, Matrix4x4 iwi, Vector3 r)
	{
		Matrix4x4 dia = Diagonal(w);
		Matrix4x4 mat = Cross(r);
		mat = mat * iwi * mat;
		mat.SetRow(0, new Vector4(w, 0, 0, 0) - mat.GetRow(0));
		mat.SetRow(1, new Vector4(0, w, 0, 0) - mat.GetRow(1));
		mat.SetRow(2, new Vector4(0, 0, w, 0) - mat.GetRow(2));
		return mat;
	}

	private Matrix4x4 Diagonal(float f)
	{
		Matrix4x4 mat = new Matrix4x4();
		mat[0, 0] = mat[1, 1] = mat[2, 2] = f;
		return mat;
	}

	private Matrix4x4 Cross(Vector3 v)
	{
		Matrix4x4 mat = new Matrix4x4();
		mat.SetRow(0, new Vector4(0, -v.z, v.y, 0));
		mat.SetRow(1, new Vector4(v.z, 0, -v.x, 0));
		mat.SetRow(2, new Vector4(-v.y, v.x, 0, 0));
		return mat;
	}

	/// <summary>
	/// 通过梯度下降法近似地找地形上距离给定点最近的点。
	/// 有很多待优化的地方。
	/// </summary>
	/// <param name="closetPoint">地形上的最近点</param>
	/// <param name="maxIters">最大迭代次数</param>
	/// <param name="stepX">x方向的采样距离</param>
	/// <param name="stepZ">z方向的采样距离</param>
	/// <returns></returns>
	static public float GradientDescent(Terrain terrain, Vector3 origin, out Vector3 closetPoint, int maxIters = 50, int stepX = 1, int stepZ = 1)
	{
		closetPoint = origin;
		closetPoint.y = terrain.SampleHeight(origin);
		float minSqrdst = Mathf.Abs(closetPoint.y - origin.y);
		minSqrdst *= minSqrdst;
		for (int i = 0; i < maxIters; i++) {
			Vector3 cp = closetPoint;
			bool hasless = false;
			for (int x = -stepX; x < stepX; x += stepX) {
				Vector3 point = cp;
				point.x = closetPoint.x + x;
				for (int z = -stepZ; z < stepZ; z += stepZ) {
					point.z = closetPoint.z + z;
					point.y = terrain.SampleHeight(point);
					float sqrdst = (origin - point).sqrMagnitude;
					if (sqrdst < minSqrdst) {
						minSqrdst = sqrdst;
						cp = point;
						hasless = true;
					}
				}
			}
			closetPoint = cp;
			if (!hasless) {
				break;
			}
		}
		return Mathf.Sqrt(minSqrdst);
	}

}
