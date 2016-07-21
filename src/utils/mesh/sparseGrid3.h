
#ifndef SPARSEGRID3D_H_
#define SPARSEGRID3D_H_
#include<unordered_map>
namespace std{
template<>
struct hash<int3> : public unary_function<int3, size_t>{
	size_t operator()(const int3& v) const {
		//TODO larger prime number (64 bit) to match size_t
		const size_t p0 = 73856093;
		const size_t p1 = 19349669;
		const size_t p2 = 83492791;
		const size_t res = ((size_t)v.x * p0) ^ ((size_t)v.y * p1) ^ ((size_t)v.z * p2);
		return res;
	}
};
}
namespace ml{
	template<class T> class SparseGrid3;
	template<class T> std::ostream& operator<<(std::ostream& s, const SparseGrid3<T>& g);

	template<class T>
	class SparseGrid3 {
	public:
		typedef typename std::unordered_map<int3, T, std::hash<int3>>::iterator iterator;
		typedef typename std::unordered_map<int3, T, std::hash<int3>>::const_iterator const_iterator;
		iterator begin() { return m_Data.begin(); }
		iterator end() { return m_Data.end(); }
		const_iterator begin() const { return m_Data.begin(); }
		const_iterator end() const { return m_Data.end(); }

		SparseGrid3(float maxLoadFactor = 0.6, size_t reserveBuckets = 64) {
			m_Data.reserve(reserveBuckets);
			m_Data.max_load_factor(maxLoadFactor);
		}

		void clear() {
			m_Data.clear();
		}

		bool exists(const int3& i) const {
			return (m_Data.find(i) != m_Data.end());
		}

		const T& operator()(const int3& i) const {
			return m_Data.find(i)->second;
		}

		//! if the element does not exist, it will be created with its default constructor
		T& operator()(const int3& i) {
			return m_Data[i];
		}

		const T& operator()(int x, int y, int z) const {
			return (*this)(make_int3(x, y, z));
		}
		T& operator()(int x, int y, int z) {
			return (*this)(make_int3(x, y, z));
		}

		const T& operator[](const int3& i) const {
			return (*this)(i);
		}
		T& operator[](const int3& i) {
			return (*this)(i);
		}

#ifdef _WIN32
		template<class U>
#endif
		friend std::ostream& operator<< <> (std::ostream& s, const SparseGrid3<T>& g);

		/*
	#ifdef _WIN32
	template<class U, class V, class W>
	#endif
	friend BinaryDataStream<U, V>& operator>> <>(BinaryDataStream<U, V>& s, SparseGrid3<T>& g);
	#ifdef _WIN32
	template<class U, class V, class W>
	#endif
	friend BinaryDataStream<U, V>& operator<< <>(BinaryDataStream<U, V>& s, const SparseGrid3<T>& g);



	void writeBinaryDump(const std::string& s) const {
	std::ofstream fout(s, std::ios::binary);
	size_t size = m_Data.size();
	float maxLoadFactor = m_Data.max_load_factor();
	fout.write((const char*)&size, sizeof(size_t));
	fout.write((const char*)&maxLoadFactor, sizeof(float));
	for (auto iter = begin(); iter != end(); iter++) {
	const ml::int3 first = iter->first;
	const T second = iter->second;
	fout.write((const char*)&first, sizeof(ml::int3));
	fout.write((const char*)&second, sizeof(T));
	}
	fout.close();
	}

	void readBinaryDump(const std::string& s) {
	m_Data.clear();
	std::ifstream fin(s, std::ios::binary);
	if (!fin.is_open()) throw MLIB_EXCEPTION("file not found " + s);
	size_t size; float maxLoadFactor;
	fin.read((char*)&size, sizeof(size_t));
	fin.read((char*)&maxLoadFactor, sizeof(float));
	for (size_t i = 0; i < size; i++) {
	ml::int3 first; T second;
	fin.read((char*)&first, sizeof(ml::int3));
	assert(fin.good());
	fin.read((char*)&second, sizeof(T));
	assert(fin.good());
	m_Data[first] = second;
	}
	fin.close();
	}



	*/


	protected:
		std::unordered_map<int3, T, std::hash<int3>> m_Data;
	};

	template<class T>
	inline std::ostream& operator<<(std::ostream& s, const SparseGrid3<T>& g) {
		for (auto iter = g.m_Data.begin(); iter != g.m_Data.end(); iter++) {
			s << "\t" << iter->first << "\t: " << iter->second << std::endl;
		}
		return s;
	}
}//end namespace

/*	//! read from binary stream overload
	template<class BinaryDataBuffer, class BinaryDataCompressor, class T>
	inline BinaryDataStream<BinaryDataBuffer, BinaryDataCompressor>& operator>>(BinaryDataStream<BinaryDataBuffer, BinaryDataCompressor>& s, SparseGrid3<T>& g) {
		g.clear();
		size_t size;	float maxLoadFactor;
		s >> size >> maxLoadFactor;
		g.m_Data.max_load_factor(maxLoadFactor);
		for (size_t i = 0; i < size; i++) {
			int3 first;	T second;
			s >> first >> second;
			g[first] = second;
		}
		return s;
	}
	//! write to binary stream overload
	template<class BinaryDataBuffer, class BinaryDataCompressor, class T>
	inline BinaryDataStream<BinaryDataBuffer, BinaryDataCompressor>& operator<<(BinaryDataStream<BinaryDataBuffer, BinaryDataCompressor>& s, const SparseGrid3<T>& g) {
		s << g.m_Data.size();
		s << g.m_Data.max_load_factor();
		for (auto iter = g.begin(); iter != g.end(); iter++) {
			s << iter->first << iter->second;
		}
		return s;
	}

	*/
#endif  // CORE_UTIL_SPARSEGRID3D_H_