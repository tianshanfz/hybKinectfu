#ifndef MESHDATA_CPP
#define MESHDATA_CPP
#include"meshData.h"
namespace ml{
	template<class FloatType>
	std::ostream& operator<<(std::ostream& os, const MeshData<FloatType>& meshData)
	{
		os << "MeshData:\n"
			<< "\tVertices:  " << meshData.m_Vertices.size() << "\n"
			<< "\tColors:    " << meshData.m_Colors.size() << "\n"
			<< "\tNormals:   " << meshData.m_Normals.size() << "\n"
			<< "\tTexCoords: " << meshData.m_TextureCoords.size() << "\n"
			<< std::endl;

		return os;
	}

	static inline bool FaceLess(const std::vector<unsigned int>& t0_, const std::vector<unsigned int>& t1_)
	{
		if (t0_.size() != t1_.size())
			return t0_.size() < t1_.size();
		else
		{
			std::vector<unsigned int> t0 = t0_;
			std::vector<unsigned int> t1 = t1_;

			std::sort(t0.begin(), t0.end());
			std::sort(t1.begin(), t1.end());
			for (size_t i = 0; i < t0.size(); i++)
			{
				if (t0[i] == t1[i]) continue;
				return t0[i] < t1[i];
			}
		}
		return false;
	}




	template <class FloatType>
	unsigned int MeshData<FloatType>::removeDuplicateFaces()
	{
		struct vecHash
		{
			size_t operator()(const std::vector<unsigned int>& v) const
			{
				//TODO larger prime number (64 bit) to match size_t
				const size_t p[] = { 73856093, 19349669, 83492791 };
				size_t res = 0;
				for (unsigned int i : v)
				{
					res = res ^ (size_t)i * p[i % 3];
				}
				return res;
				//const size_t res = ((size_t)v.x * p0)^((size_t)v.y * p1)^((size_t)v.z * p2);
			}
		};

		size_t numFaces = m_FaceIndicesVertices.size();
		Indices faces_new;
		faces_new.reserve(numFaces);

		unsigned int count = 0; unsigned int ordered = 0;
		std::unordered_set<std::vector<unsigned int>, vecHash> _set;
		for (size_t i = 0; i < numFaces; i++) {
			//Indices::Face f = m_FaceIndicesVertices[i]; // same pointer
			std::vector<unsigned int> face(m_FaceIndicesVertices[i].size()); // copy so that m_FaceIndicesVertices[i] remains unsorted
			for (unsigned int j = 0; j < m_FaceIndicesVertices[i].size(); j++)
				face[j] = m_FaceIndicesVertices[i][j];
			std::sort(face.begin(), face.end());
			if (_set.find(face) == _set.end()) {
				//not found yet
				_set.insert(face);
				faces_new.push_back(m_FaceIndicesVertices[i]);	//inserted the unsorted one
			}
		}
		if (m_FaceIndicesVertices.size() != faces_new.size()) {
			m_FaceIndicesVertices = Indices(faces_new);
		}

		//std::cout << "Removed " << numFaces-faces_new.size() << " duplicate faces of " << numFaces << " faces" << std::endl;

		return (unsigned int)faces_new.size();
	}




	static inline bool VertexLess(const float3& v0, const float3& v1)
	{
		if (v0.x < v1.x) return true;
		if (v0.x> v1.x) return false;
		if (v0.y< v1.y) return true;
		if (v0.y > v1.y) return false;
		if (v0.z < v1.z) return true;

		return false;
	}


	template <class FloatType>
	unsigned int MeshData<FloatType>::removeDuplicateVertices() {
		unsigned int numV = (unsigned int)m_Vertices.size();
		//int numT = (int)tris.size();

		std::map<float3, unsigned int, bool(*)(const float3&, const float3&)> pts(VertexLess);

		std::vector<unsigned int> vertexLookUp;	vertexLookUp.resize(numV);
		std::vector<float3> new_verts; new_verts.reserve(numV);
		std::vector<float4> new_color;		if (hasPerVertexColors())		new_color.reserve(m_Colors.size());
		std::vector<float3> new_normals;	if (hasPerVertexNormals())		new_normals.reserve(m_Normals.size());
		std::vector<float2> new_tex;		if (hasPerVertexTexCoords())	new_tex.reserve(m_TextureCoords.size());

		unsigned int cnt = 0;
		for (size_t i1 = 0; i1 < numV; i1++) {
			const float3& pt = m_Vertices[i1];

			std::map<float3, unsigned int, bool(*)(const float3&, const float3&) >::iterator it = pts.find(pt);

			if (it != pts.end()) {
				vertexLookUp[i1] = it->second;
			}
			else {
				pts.insert(std::make_pair(pt, cnt));
				new_verts.push_back(pt);
				vertexLookUp[i1] = cnt;
				cnt++;
				if (hasPerVertexColors())		new_color.push_back(m_Colors[i1]);
				if (hasPerVertexNormals())		new_normals.push_back(m_Normals[i1]);
				if (hasPerVertexTexCoords())	new_tex.push_back(m_TextureCoords[i1]);
			}
		}

		// Update faces
		for (auto it = m_FaceIndicesVertices.begin(); it != m_FaceIndicesVertices.end(); it++) {
			for (auto idx = it->begin(); idx != it->end(); idx++) {
				*idx = vertexLookUp[*idx];
			}
			//*it = vertexLookUp[*it];
		}

		//std::cout << "Removed " << numV-cnt << " duplicate vertices of " << numV << " vertices" << std::endl;

		if (m_Vertices != new_verts) {
			m_Vertices = std::vector<float3>(new_verts.begin(), new_verts.end());
			if (hasPerVertexColors())		m_Colors = std::vector<float4>(new_color.begin(), new_color.end());
			if (hasPerVertexNormals())		m_Normals = std::vector<float3>(new_normals.begin(), new_normals.end());
			if (hasPerVertexTexCoords())	m_TextureCoords = std::vector<float2>(new_tex.begin(), new_tex.end());
		}

		return cnt;
	}



	template <class FloatType>
	unsigned int MeshData<FloatType>::hasNearestNeighbor(const int3& coord, SparseGrid3<std::list<std::pair<float3, unsigned int> > > &neighborQuery, const float3& v, FloatType thresh)
	{
		FloatType threshSq = thresh*thresh;
		for (int i = -1; i <= 1; i++) {
			for (int j = -1; j <= 1; j++) {
				for (int k = -1; k <= 1; k++) {
					int3 c = coord + make_int3(i, j, k);
					if (neighborQuery.exists(c)) {
						for (const std::pair<float3, unsigned int>& n : neighborQuery[c]) {
							if (dot(v - n.first, v - n.first) < threshSq) {
								return n.second;
							}
						}
					}
				}
			}
		}
		return (unsigned int)-1;
	}

	template <class FloatType>
	unsigned int MeshData<FloatType>::hasNearestNeighborApprox(const int3& coord, SparseGrid3<unsigned int> &neighborQuery, FloatType thresh) {
		FloatType threshSq = thresh*thresh;

		for (int i = -1; i <= 1; i++) {
			for (int j = -1; j <= 1; j++) {
				for (int k = -1; k <= 1; k++) {
					int3 c = coord + make_int3(i, j, k);
					if (neighborQuery.exists(c)) {
						return neighborQuery[c];
					}
				}
			}
		}
		return (unsigned int)-1;
	}



	template <class FloatType>
	unsigned int MeshData<FloatType>::mergeCloseVertices(FloatType thresh, bool approx)
	{
		if (thresh <= (FloatType)0)	
			cout << "invalid thresh " << std::to_string(thresh);
		unsigned int numV = (unsigned int)m_Vertices.size();

		std::vector<unsigned int> vertexLookUp;	vertexLookUp.resize(numV);
		std::vector<float3> new_verts; new_verts.reserve(numV);
		std::vector<float4> new_color;		if (hasPerVertexColors())		new_color.reserve(m_Colors.size());
		std::vector<float3> new_normals;	if (hasPerVertexNormals())		new_normals.reserve(m_Normals.size());
		std::vector<float2> new_tex;		if (hasPerVertexTexCoords())	new_tex.reserve(m_TextureCoords.size());

		unsigned int cnt = 0;
		if (approx)
		{
			SparseGrid3<unsigned int> neighborQuery(0.6f, numV * 2);
			for (unsigned int v = 0; v < numV; v++)
			{
				const float3& vert = m_Vertices[v];
				int3 coord = toVirtualVoxelPos(vert, thresh);
				unsigned int nn = hasNearestNeighborApprox(coord, neighborQuery, thresh);

				if (nn == (unsigned int)-1) 
				{
					neighborQuery[coord] = cnt;
					new_verts.push_back(vert);
					vertexLookUp[v] = cnt;
					cnt++;
					if (hasPerVertexColors())		
						new_color.push_back(m_Colors[v]);
					if (hasPerVertexNormals())		
						new_normals.push_back(m_Normals[v]);
					if (hasPerVertexTexCoords())	
						new_tex.push_back(m_TextureCoords[v]);
				}
				else
				{
					vertexLookUp[v] = nn;
				}
			}
		}
		else
		{
			SparseGrid3<std::list<std::pair<float3, unsigned int> > > neighborQuery(0.6f, numV * 2);
			for (unsigned int v = 0; v < numV; v++)
			{
				const float3& vert = m_Vertices[v];
				int3 coord = toVirtualVoxelPos(vert, thresh);
				unsigned int nn = hasNearestNeighbor(coord, neighborQuery, vert, thresh);

				if (nn == (unsigned int)-1)
				{
					neighborQuery[coord].push_back(std::make_pair(vert, cnt));
					new_verts.push_back(vert);
					vertexLookUp[v] = cnt;
					cnt++;
					if (hasPerVertexColors())		new_color.push_back(m_Colors[v]);
					if (hasPerVertexNormals())		new_normals.push_back(m_Normals[v]);
					if (hasPerVertexTexCoords())	new_tex.push_back(m_TextureCoords[v]);
				}
				else
				{
					vertexLookUp[v] = nn;
				}
			}
		}

		// Update faces
		for (auto it = m_FaceIndicesVertices.begin(); it != m_FaceIndicesVertices.end(); it++) {
			for (auto idx = it->begin(); idx != it->end(); idx++) {
				*idx = vertexLookUp[*idx];
			}
		}

		if (m_Vertices.size() != new_verts.size()) 
		{
			m_Vertices = std::vector<float3>(new_verts.begin(), new_verts.end());

			if (hasPerVertexColors())		m_Colors = std::vector<float4>(new_color.begin(), new_color.end());
			if (hasPerVertexNormals())		m_Normals = std::vector<float3>(new_normals.begin(), new_normals.end());
			if (hasPerVertexTexCoords())	m_TextureCoords = std::vector<float2>(new_tex.begin(), new_tex.end());
		}

		removeDegeneratedFaces();
		//std::cout << "Merged " << numV-cnt << " of " << numV << " vertices" << std::endl;
		return cnt;
	}



	template <class FloatType>
	unsigned int MeshData<FloatType>::removeDegeneratedFaces()
	{
		Indices newFacesIndicesVertices;

		for (size_t i = 0; i < m_FaceIndicesVertices.size(); i++) {
			std::unordered_set<unsigned int> _set(m_FaceIndicesVertices[i].size());
			bool foundDuplicate = false;
			for (unsigned int idx : m_FaceIndicesVertices[i]) {
				if (_set.find(idx) != _set.end()) {
					foundDuplicate = true;
					break;
				}
				else {
					_set.insert(idx);
				}
			}
			if (!foundDuplicate) {
				newFacesIndicesVertices.push_back(m_FaceIndicesVertices[i]);
			}
		}
		if (m_FaceIndicesVertices.size() != newFacesIndicesVertices.size()) {
			m_FaceIndicesVertices = newFacesIndicesVertices;
		}

		return (unsigned int)m_FaceIndicesVertices.size();
	}




	template <class FloatType>
	unsigned int MeshData<FloatType>::removeIsolatedVertices()
	{
		unsigned int numV = (unsigned int)m_Vertices.size();
		std::vector<unsigned int> vertexLookUp;	vertexLookUp.resize(numV);
		std::vector<float3> new_verts; new_verts.reserve(numV);
		std::vector<float4> new_color;		if (hasPerVertexColors())		new_color.reserve(m_Colors.size());
		std::vector<float3> new_normals;	if (hasPerVertexNormals())		new_normals.reserve(m_Normals.size());
		std::vector<float2> new_tex;		if (hasPerVertexTexCoords())	new_tex.reserve(m_TextureCoords.size());

		std::unordered_map<unsigned int, unsigned int> _map(m_Vertices.size());
		unsigned int cnt = 0;
		for (auto& face : m_FaceIndicesVertices) {
			for (auto& idx : face) {
				if (_map.find(idx) != _map.end()) {
					idx = _map[idx];	//set to new idx, which already exists
				}
				else {
					_map[idx] = cnt;
					new_verts.push_back(m_Vertices[idx]);
					if (hasPerVertexColors())		new_color.push_back(m_Colors[idx]);
					if (hasPerVertexNormals())		new_normals.push_back(m_Normals[idx]);
					if (hasPerVertexTexCoords())	new_tex.push_back(m_TextureCoords[idx]);

					idx = cnt;
					cnt++;
				}
			}
		}

		m_Vertices = std::vector<float3>(new_verts.begin(), new_verts.end());

		if (hasPerVertexColors())		m_Colors = std::vector<float4>(new_color.begin(), new_color.end());
		if (hasPerVertexNormals())		m_Normals = std::vector<float3>(new_normals.begin(), new_normals.end());
		if (hasPerVertexTexCoords())	m_TextureCoords = std::vector<float2>(new_tex.begin(), new_tex.end());

		return (unsigned int)m_Vertices.size();
	}


	template <class FloatType>
	unsigned int MeshData<FloatType>::removeVerticesInFrontOfPlane(const Plane<FloatType>& plane, FloatType thresh)
	{
		unsigned int numV = (unsigned int)m_Vertices.size();
		unsigned int numF = (unsigned int)m_FaceIndicesVertices.size();

		std::vector<unsigned int> vertexLookUp;	vertexLookUp.resize(numV);
		std::vector<float3> new_verts;	new_verts.reserve(numV);
		Indices new_faces;	new_faces.reserve(numF);
		std::vector<float4> new_color;		if (hasPerVertexColors())		new_color.reserve(m_Colors.size());
		std::vector<float3> new_normals;	if (hasPerVertexNormals())		new_normals.reserve(m_Normals.size());
		std::vector<float2> new_tex;		if (hasPerVertexTexCoords())	new_tex.reserve(m_TextureCoords.size());

		std::unordered_map<unsigned int, unsigned int> _map(m_Vertices.size());
		unsigned int cnt = 0;
		for (auto& face : m_FaceIndicesVertices) {
			bool keepFace = true;
			for (auto& idx : face) {
				if (plane.distanceToPoint(m_Vertices[idx]) > thresh) {
					keepFace = false;
					break;
				}
			}
			if (keepFace) {
				for (auto& idx : face) {

					if (_map.find(idx) != _map.end()) {
						idx = _map[idx];	//set to new idx, which already exists
					}
					else {
						_map[idx] = cnt;
						new_verts.push_back(m_Vertices[idx]);
						if (hasPerVertexColors())		new_color.push_back(m_Colors[idx]);
						if (hasPerVertexNormals())		new_normals.push_back(m_Normals[idx]);
						if (hasPerVertexTexCoords())	new_tex.push_back(m_TextureCoords[idx]);

						idx = cnt;
						cnt++;
					}
				}
				new_faces.push_back(face);
			}
		}

		m_Vertices = std::vector<float3>(new_verts.begin(), new_verts.end());

		if (hasPerVertexColors())		m_Colors = std::vector<float4>(new_color.begin(), new_color.end());
		if (hasPerVertexNormals())		m_Normals = std::vector<float3>(new_normals.begin(), new_normals.end());
		if (hasPerVertexTexCoords())	m_TextureCoords = std::vector<float2>(new_tex.begin(), new_tex.end());

		m_FaceIndicesVertices = new_faces;

		return (unsigned int)m_Vertices.size();
	}



	template <class FloatType>
	unsigned int MeshData<FloatType>::removeFacesInFrontOfPlane(const Plane<FloatType>& plane, FloatType thresh /*= 0.0f*/)
	{
		unsigned int numV = (unsigned int)m_Vertices.size();
		unsigned int numF = (unsigned int)m_FaceIndicesVertices.size();

		std::vector<unsigned int> vertexLookUp;	vertexLookUp.resize(numV);
		std::vector<float3> new_verts;	new_verts.reserve(numV);
		std::vector<std::vector<unsigned int>> new_faces;	new_faces.reserve(numF);
		std::vector<float4> new_color;		if (hasPerVertexColors())		new_color.reserve(m_Colors.size());
		std::vector<float3> new_normals;	if (hasPerVertexNormals())		new_normals.reserve(m_Normals.size());
		std::vector<float2> new_tex;		if (hasPerVertexTexCoords())	new_tex.reserve(m_TextureCoords.size());

		std::unordered_map<unsigned int, unsigned int> _map(m_Vertices.size());
		unsigned int cnt = 0;
		for (auto& face : m_FaceIndicesVertices) {
			bool keepFace = false;
			for (auto& idx : face) {
				if (plane.distanceToPoint(m_Vertices[idx]) <= thresh) {
					keepFace = true;
					break;
				}
			}
			if (keepFace) {
				for (auto& idx : face) {

					if (_map.find(idx) != _map.end()) {
						idx = _map[idx];	//set to new idx, which already exists
					}
					else {
						_map[idx] = cnt;
						new_verts.push_back(m_Vertices[idx]);
						if (hasPerVertexColors())		new_color.push_back(m_Colors[idx]);
						if (hasPerVertexNormals())		new_normals.push_back(m_Normals[idx]);
						if (hasPerVertexTexCoords())	new_tex.push_back(m_TextureCoords[idx]);

						idx = cnt;
						cnt++;
					}
				}
				new_faces.push_back(face);
			}
		}

		m_Vertices = std::vector<float3>(new_verts.begin(), new_verts.end());

		if (hasPerVertexColors())		m_Colors = std::vector<float4>(new_color.begin(), new_color.end());
		if (hasPerVertexNormals())		m_Normals = std::vector<float3>(new_normals.begin(), new_normals.end());
		if (hasPerVertexTexCoords())	m_TextureCoords = std::vector<float2>(new_tex.begin(), new_tex.end());

		m_FaceIndicesVertices = std::vector<std::vector<unsigned int>>(new_faces.begin(), new_faces.end());

		return (unsigned int)m_Vertices.size();
	}


	template <class FloatType>
	void MeshData<FloatType>::merge(const MeshData<FloatType>& other)
	{
		if (other.isEmpty()) {
			return;
		}
		if (isEmpty()) {
			*this = other;
			return;
		}


		if (hasVertexIndices() != other.hasVertexIndices())
		{
				cout<<("invalid mesh conversion");
				return;
		}

		if (hasNormals() != other.hasNormals() || hasNormalIndices() != other.hasNormalIndices()) {
			m_Normals.clear();
			m_FaceIndicesNormals.clear();
		}
		if (hasColors() != other.hasColors() || hasColorIndices() != other.hasColorIndices()) {
			m_Colors.clear();
			m_FaceIndicesColors.clear();
		}
		if (hasTexCoords() != other.hasTexCoords() || hasTexCoordsIndices() != other.hasTexCoordsIndices()) {
			m_TextureCoords.clear();
			m_FaceIndicesTextureCoords.clear();
		}

		size_t vertsBefore = m_Vertices.size();
		size_t normsBefore = m_Normals.size();
		size_t colorBefore = m_Colors.size();
		size_t texCoordsBefore = m_TextureCoords.size();
		m_Vertices.insert(m_Vertices.end(), other.m_Vertices.begin(), other.m_Vertices.end());
		if (hasColors())	m_Colors.insert(m_Colors.end(), other.m_Colors.begin(), other.m_Colors.end());
		if (hasNormals())	m_Normals.insert(m_Normals.end(), other.m_Normals.begin(), other.m_Normals.end());
		if (hasTexCoords())	m_TextureCoords.insert(m_TextureCoords.end(), other.m_TextureCoords.begin(), other.m_TextureCoords.end());

		if (hasVertexIndices()) {
			size_t indicesBefore = m_FaceIndicesVertices.size();
			//m_FaceIndicesVertices.insert(m_FaceIndicesVertices.end(), other.m_FaceIndicesVertices.begin(), other.m_FaceIndicesVertices.end());
			m_FaceIndicesVertices.append(other.m_FaceIndicesVertices);
			for (size_t i = indicesBefore; i < m_FaceIndicesVertices.size(); i++) {
				for (auto& idx : m_FaceIndicesVertices[i]) idx += (unsigned int)vertsBefore;
			}
		}
		if (hasNormalIndices()) {
			size_t indicesBefore = m_FaceIndicesNormals.size();
			//m_FaceIndicesNormals.insert(m_FaceIndicesNormals.end(), other.m_FaceIndicesNormals.begin(), other.m_FaceIndicesNormals.end());
			m_FaceIndicesNormals.append(other.m_FaceIndicesNormals);
			for (size_t i = indicesBefore; i < m_FaceIndicesNormals.size(); i++) {
				for (auto& idx : m_FaceIndicesNormals[i]) idx += (unsigned int)normsBefore;
			}
		}
		if (hasColorIndices()) {
			size_t indicesBefore = m_FaceIndicesColors.size();
			//m_FaceIndicesColors.insert(m_FaceIndicesColors.end(), other.m_FaceIndicesColors.begin(), other.m_FaceIndicesColors.end());
			m_FaceIndicesColors.append(other.m_FaceIndicesColors);
			for (size_t i = indicesBefore; i < m_FaceIndicesColors.size(); i++) {
				for (auto& idx : m_FaceIndicesColors[i]) idx += (unsigned int)colorBefore;
			}
		}
		if (hasTexCoordsIndices()) {
			size_t indicesBefore = m_FaceIndicesTextureCoords.size();
			//m_FaceIndicesTextureCoords.insert(m_FaceIndicesTextureCoords.end(), other.m_FaceIndicesTextureCoords.begin(), other.m_FaceIndicesTextureCoords.end());
			m_FaceIndicesTextureCoords.append(other.m_FaceIndicesTextureCoords);
			for (size_t i = indicesBefore; i < m_FaceIndicesTextureCoords.size(); i++) {
				for (auto& idx : m_FaceIndicesTextureCoords[i]) idx += (unsigned int)texCoordsBefore;
			}
		}
	}


	template <class FloatType>
	void MeshData<FloatType>::subdivideFacesMidpoint()
	{
		m_Vertices.reserve(m_Vertices.size() + m_FaceIndicesVertices.size());	//there will be 1 new vertex per face
		if (hasPerVertexColors())		m_Colors.reserve(m_Colors.size() + m_FaceIndicesVertices.size());
		if (hasPerVertexNormals())		m_Normals.reserve(m_Normals.size() + m_FaceIndicesVertices.size());
		if (hasPerVertexTexCoords())	m_TextureCoords.reserve(m_TextureCoords.size() + m_FaceIndicesVertices.size());

		Indices newFaces;
		for (auto& face : m_FaceIndicesVertices) {
			float3 centerP = make_float3(0, 0, 0);
			for (auto& idx : face) {
				centerP += m_Vertices[idx];
			}
			centerP /= (FloatType)face.size();
			m_Vertices.push_back(centerP);

			if (hasPerVertexColors()) {
				float4 centerC = make_float4(0, 0, 0, 0);
				for (auto& idx : face) {
					centerC += m_Colors[idx];
				}
				centerC /= (FloatType)face.size();
				m_Colors.push_back(centerC);
			}
			if (hasPerVertexNormals()) {
				float3 centerN = make_float3(0, 0, 0);
				for (auto& idx : face) {
					centerN += m_Normals[idx];
				}
				centerN /= (FloatType)face.size();
				m_Normals.push_back(centerN);
			}
			if (hasPerVertexTexCoords()) {
				float2 centerT = make_float2(0, 0);
				for (auto& idx : face) {
					centerT += m_TextureCoords[idx];
				}
				centerT /= (FloatType)face.size();
				m_TextureCoords.push_back(centerT);
			}


			unsigned int newIdx = (unsigned int)m_Vertices.size() - 1;
			for (unsigned int i = 0; i < face.size(); i++) {
				newFaces.push_back(std::vector<unsigned int>(3));
				newFaces[newFaces.size() - 1][0] = face[i];
				newFaces[newFaces.size() - 1][1] = face[(i + 1) % face.size()];
				newFaces[newFaces.size() - 1][2] = newIdx;
			}
		}

		m_FaceIndicesVertices = newFaces;
	}


	template <class FloatType>
	FloatType MeshData<FloatType>::subdivideFacesLoop(float edgeThresh /*= 0.0f*/)
	{
		m_Vertices.reserve(m_Vertices.size() + m_FaceIndicesVertices.size());	//there will be 1 new vertex per face (TODO FIX)
		if (hasPerVertexColors())		m_Colors.reserve(m_Colors.size() + m_FaceIndicesVertices.size());
		if (hasPerVertexNormals())		m_Normals.reserve(m_Normals.size() + m_FaceIndicesVertices.size());
		if (hasPerVertexTexCoords())	m_TextureCoords.reserve(m_TextureCoords.size() + m_FaceIndicesVertices.size());


		struct Edge {
			Edge(unsigned int _v0, unsigned int _v1) {
				if (_v0 < _v1) {
					v0 = _v0;
					v1 = _v1;
				}
				else {
					v1 = _v0;
					v0 = _v1;
				}
			}
			bool operator==(const Edge& other) const {
				return v0 == other.v0 && v1 == other.v1;
			}

			bool needEdgeVertex(float thresh, const std::vector<float3>& vertices) const {
				if (thresh == 0.0f) return true;
				else {
					return (dot(vertices[v0] - vertices[v1], vertices[v0] - vertices[v1])> thresh*thresh);
				}
			}

			float edgeLength(const std::vector<float3>& vertices) const {
				return norm(vertices[v0] - vertices[v1]);
			}

			unsigned int v0;
			unsigned int v1;
		};

		struct EdgeHash {
			size_t operator()(const Edge& e) const {
				//TODO larger prime number (64 bit) to match size_t
				const size_t p[] = { 73856093, 19349669 };
				return e.v0*p[0] ^ e.v1*p[1];
				//const size_t res = ((size_t)v.x * p0)^((size_t)v.y * p1)^((size_t)v.z * p2);
			}
		};

		FloatType maxEdgeLen = 0.0f;

		//maps edges to new vertex indices
		std::unordered_map<Edge, unsigned int, EdgeHash> edgeMap;
		for (auto& face : m_FaceIndicesVertices) {

			for (unsigned int i = 0; i < face.size(); i++) {
				Edge e(face[i], face[(i + 1) % face.size()]);

				FloatType edgeLen = e.edgeLength(m_Vertices);
				if (edgeLen > maxEdgeLen) maxEdgeLen = edgeLen;

				if (e.needEdgeVertex(edgeThresh, m_Vertices)) {
					if (edgeMap.find(e) == edgeMap.end()) {
						m_Vertices.push_back((FloatType)0.5*(m_Vertices[e.v0] + m_Vertices[e.v1]));
						if (hasPerVertexColors()) m_Colors.push_back((FloatType)0.5*(m_Colors[e.v0] + m_Colors[e.v1]));
						if (hasPerVertexNormals()) m_Normals.push_back((FloatType)0.5*(m_Normals[e.v0] + m_Normals[e.v1]));
						if (hasPerVertexTexCoords()) m_TextureCoords.push_back((FloatType)0.5*(m_TextureCoords[e.v0] + m_TextureCoords[e.v1]));
						unsigned int idx = (unsigned int)m_Vertices.size() - 1;
						edgeMap[e] = idx;
					}
				}
			}

		}

		Indices newFaces;    newFaces.reserve(m_FaceIndicesVertices.size() * 4);
		for (auto& face : m_FaceIndicesVertices) {
			bool allEdgesExist = true;
			bool noneEdgesExist = true;
			for (unsigned int i = 0; i < face.size(); i++) {
				Edge e(face[i], face[(i + 1) % face.size()]);
				if (edgeMap.find(e) == edgeMap.end())   {
					allEdgesExist = false;
				}
				else {
					noneEdgesExist = false;
				}
			}

			if (allEdgesExist) {
				std::vector<unsigned int> centerFace(face.size());
				for (unsigned int i = 0; i < face.size(); i++) {
					Edge ePrev(face[i], face[(i + 1) % face.size()]);
					Edge eNext(face[(i + 1) % face.size()], face[(i + 2) % face.size()]);

					unsigned int curr[] = {
						edgeMap[ePrev],
						face[(i + 1) % face.size()],
						edgeMap[eNext]
					};
					newFaces.addFace(curr, 3);
					centerFace[i] = curr[0];
					//newFaces.push_back(std::vector<unsigned int>(3));
					//newFaces.back()[0] = edgeMap[ePrev];
					//newFaces.back()[1] = face[(i+1)%face.size()];
					//newFaces.back()[2] = edgeMap[eNext];

					//centerFace[i] = newFaces.back()[0];
				}
				newFaces.push_back(centerFace);

			}
			else if (noneEdgesExist) {
				newFaces.push_back(face);
			}
			else {
				std::vector<unsigned int> cFace;
				for (unsigned int i = 0; i < face.size(); i++) {
					cFace.push_back(face[i]);
					Edge e(face[i], face[(i + 1) % face.size()]);
					if (edgeMap.find(e) != edgeMap.end())   cFace.push_back(edgeMap[e]);
				}

				//centroid based vertex insertion
				float3 centerP = make_float3(0, 0, 0);
				for (auto& idx : face) {
					centerP += m_Vertices[idx];
				}
				centerP /= (FloatType)face.size();
				m_Vertices.push_back(centerP);

				if (hasPerVertexColors()) {
					float4 centerC = make_float4(0, 0, 0, 0);
					for (auto& idx : face) {
						centerC += m_Colors[idx];
					}
					centerC /= (FloatType)face.size();
					m_Colors.push_back(centerC);
				}
				if (hasPerVertexNormals()) {
					float3 centerN = make_float3(0, 0, 0);
					for (auto& idx : face) {
						centerN += m_Normals[idx];
					}
					centerN /= (FloatType)face.size();
					m_Normals.push_back(centerN);
				}
				if (hasPerVertexTexCoords()) {
					float2 centerT = make_float2(0, 0);
					for (auto& idx : face) {
						centerT += m_TextureCoords[idx];
					}
					centerT /= (FloatType)face.size();
					m_TextureCoords.push_back(centerT);
				}


				unsigned int newIdx = (unsigned int)m_Vertices.size() - 1;
				for (size_t i = 0; i < cFace.size(); i++) {
					newFaces.push_back(std::vector<unsigned int>(3));
					newFaces[newFaces.size() - 1][0] = cFace[i];
					newFaces[newFaces.size() - 1][1] = cFace[(i + 1) % cFace.size()];
					newFaces[newFaces.size() - 1][2] = newIdx;
				}
			}
		}

		//m_FaceIndicesVertices = std::vector<std::vector<unsigned int>>(newFaces.begin(), newFaces.end());
		m_FaceIndicesVertices = newFaces;
		return maxEdgeLen;
	}
}//namespace
#endif
