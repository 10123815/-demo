using System.Collections.Generic;
using UnityEngine;

public class SoftBody : MonoBehaviour
{

	public bool isTorus = true;

	public float W = 0.1f;
	public Vector3 G = new Vector3(0, -9.8f, 0);
	public float STIFFNESS = 1;
	public int INTERATIONS = 20;

	protected List<Node> m_nodes;
	protected List<Link> m_links;
	protected List<Face> m_faces;

	// 和刚体的碰撞信息
	private List<RigidContactInfo> m_rigidContactInfos;

	// 这个相当于overlapping pairs
	protected List<Collider> m_colliders;

	protected Mesh m_mesh;
	private Terrain m_terrain;

	protected class RigidContactInfo
	{
		public Matrix4x4 impMat;
		public Vector3 normal = Vector3.up;
		public float offset;
		public float friction = 0.1f;
		public float hardness = 0.1f;

		public Collider collider;
		public Node node;

		public float param0;    // dt / mass
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
		public float rest; // 弹簧的原始长度
		public float param0; // k / (w1 + w2)
	}

	protected class Face
	{
		public Node n1, n2, n3;
		public float area;      // 三角形面积
	}

	void Awake()
	{
		CreateSoftBody();
		m_colliders = new List<Collider>();

		m_terrain = GameObject.FindGameObjectWithTag("Terrain").GetComponent<Terrain>();
		m_colliders.Add(m_terrain.GetComponent<TerrainCollider>());

		m_rigidContactInfos = new List<RigidContactInfo>();
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
		for (int i = 0; i < m_nodes.Count; i++) {
			Node node = m_nodes[i];
			Debug.DrawLine(node.prevpos, node.curpos, Color.blue);
		}
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

		List<bool> linked = new List<bool>();
		for (int i = 0; i < vertices.Length * vertices.Length; i++) {
			linked.Add(false);
		}

		for (int i = 0; i < tris.Length; i += 3) {
			for (int j = 0, k = 1; j < 3; j++, k++, k %= 3) {
				int idx1 = tris[i + j];
				int idx2 = tris[i + k];
				if (!linked[idx1 * vertices.Length + idx2] && !linked[idx2 * vertices.Length + idx1]) {
					linked[idx1 * vertices.Length + idx2] = true;
					linked[idx2 * vertices.Length + idx1] = true;
					Link link = new Link();
					link.n1 = m_nodes[idx1];
					link.n2 = m_nodes[idx2];
					link.rest = (link.n1.curpos - link.n2.curpos).sqrMagnitude;
					m_links.Add(link);
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
		for (int i = 0; i < m_links.Count; i++) {
			Link link = m_links[i];
			float w = link.n1.w + link.n2.w;
			link.param0 = w / STIFFNESS;
		}

		for (int i = 0; i < m_nodes.Count; i++) {
			if (m_nodes[i].w <= 0) {
				continue;
			}
			// 重力
			m_nodes[i].velocity += G * Time.fixedDeltaTime;
			m_nodes[i].prevpos = m_nodes[i].curpos;
			m_nodes[i].curpos += m_nodes[i].velocity * Time.fixedDeltaTime;
			// TODO: 其他的外力
		}

		transform.position = m_nodes[0].curpos;

		m_rigidContactInfos.Clear();
	}

	protected void NarrowPhase()
	{
		for (int i = 0; i < m_colliders.Count; i++) {
			Collider collider = m_colliders[i];
			for (int j = 0; j < m_nodes.Count; j++) {
				// 检测各个顶点是否与collider碰撞。不精确，有时三个点不在collider内，但是三角形却与collider相交
				Node node = m_nodes[j];
				if (collider.tag != "Terrain") {
					// 最近点
					Vector3 point = collider.ClosestPoint(node.curpos);
				}
				else {
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
							// 可以通过梯度下降法求一个近似的最近点，限制最多迭代次数
							Vector3 closetPoint;
							float dst = -GradientDescent(m_terrain, node.curpos, out closetPoint);
							if (Physics.Raycast(node.curpos, closetPoint - node.curpos, out hit, layerMask)) {
								rci.normal = hit.normal;
							}
							rci.node = node;
							rci.collider = collider;
							rci.param0 = node.w * Time.fixedDeltaTime;
							rci.offset = -Vector3.Dot(rci.normal, node.curpos - rci.normal * dst);
							rci.impMat = ImpulseMatrix(node.w, 0, Matrix4x4.zero, node.curpos);
							m_rigidContactInfos.Add(rci);
						}
						else if (Physics.Linecast(node.prevpos, node.curpos, out hit, layerMask)) {
							// 外→内
							float dst = -(node.curpos - hit.point).magnitude;
							rci.normal = hit.normal.normalized;
							rci.node = node;
							rci.collider = collider;
							rci.param0 = node.w * Time.fixedDeltaTime;
							rci.offset = -Vector3.Dot(rci.normal, node.curpos - rci.normal * dst);
							rci.impMat = ImpulseMatrix(node.w, 0, Matrix4x4.zero, node.curpos);
							m_rigidContactInfos.Add(rci);
						}
					}
					else if (node.prevpos.y >= y && Physics.Linecast(node.prevpos, node.curpos, out hit, layerMask)) {
						// 都在物体外，但射线穿过物体，小物体、尖锐的地方可能会出现这种情况
						float dst = -(node.curpos - hit.point).magnitude;
						rci.normal = hit.normal.normalized;
						rci.node = node;
						rci.collider = collider;
						rci.param0 = node.w * Time.fixedDeltaTime;
						rci.offset = -Vector3.Dot(rci.normal, node.curpos - rci.normal * dst);
						rci.impMat = ImpulseMatrix(node.w, 0, Matrix4x4.zero, node.curpos);
						m_rigidContactInfos.Add(rci);
					}

					// 内→外不算，因为线性约束已经把质点拉出物体了
				}
			}
		}
	}

	protected void SolveContraints()
	{

		// contacts solver
		for (int j = 0; j < m_rigidContactInfos.Count; j++) {
			RigidContactInfo rci = m_rigidContactInfos[j];
			Node node = rci.node;
			Collider collider = rci.collider;

			Vector3 va = Vector3.zero;
			if (collider.tag != "Terrain") {
				// TODO: 刚体的速度
			}
			Vector3 vb = node.curpos - node.prevpos;
			Vector3 vr = vb - va;
			float dn = Vector3.Dot(vr, rci.normal);
			float dp = Mathf.Min(Vector3.Dot(node.curpos, rci.normal) + rci.offset, 0.25f);
			Vector3 fv = vr - rci.normal * dn;
			Vector3 impulse = rci.impMat * (vr - (fv * rci.friction) + (rci.normal * dp * rci.hardness));
			node.curpos -= impulse * rci.param0;
			if (collider.tag != "Terrain") {
				// 刚体收到的冲量矩
			}
		}
		// solve position constraints
		for (int i = 0; i < INTERATIONS; i++) {
			// linear soler
			for (int j = 0; j < m_links.Count; j++) {
				Link link = m_links[j];
				if (link.param0 > 0) {
					Node n1 = link.n1;
					Node n2 = link.n2;
					Vector3 dir = n1.curpos - n2.curpos;
					float len = dir.sqrMagnitude;
					if (len > 1.192092896e-07F) {
						Vector3 dp = (len - link.rest) / (len + link.rest) / link.param0 * dir;
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
