
#ifndef MESHIO_H_
#define MESHIO_H_

//#include"cudaUtils.cuh"
#include"meshData.h"

namespace ml{
	template <class FloatType>
	class MeshIO {

	public:

		static MeshData<FloatType> loadFromFile(const std::string& filename) {
			MeshData<FloatType> data;
			loadFromFile(filename, data);
			return data;
		}
		//! gets the file extension (ignoring case)
		static std::string getFileExtension(const std::string& filename) {
			std::string extension = filename.substr(filename.find_last_of(".") + 1);
			for (unsigned int i = 0; i < extension.size(); i++) {
				extension[i] = tolower(extension[i]);
			}
			return extension;
		}
		static void loadFromFile(const std::string& filename, MeshData<FloatType>& mesh) {
			mesh.clear();
			std::string extension = getFileExtension(filename);

			if (extension == "off") {
				loadFromOFF(filename, mesh);
			}
			else if (extension == "ply") {
				loadFromPLY(filename, mesh);
			}
			else if (extension == "obj") {
				loadFromOBJ(filename, mesh);
			}
			else 	{
				cout << "unknown file format: " << filename;
			}

			mesh.deleteRedundantIndices();

			if (!mesh.isConsistent()) {
				cout << "inconsistent mesh data: " << filename;
			}
		}

		static void saveToFile(const std::string& filename, const MeshData<FloatType>& mesh) {

			if (mesh.isEmpty())
			{
				cout << "empty mesh";
				return;
			}

			if (!mesh.isConsistent()) {
				cout << "inconsistent mesh data: " << filename;
			}

			std::string extension = getFileExtension(filename);

			if (extension == "off") {
				saveToOFF(filename, mesh);
			}
			else if (extension == "ply") {
				saveToPLY(filename, mesh);
			}
			else if (extension == "obj") {
				saveToOBJ(filename, mesh);
			}
			else {
				cout << "unknown file format: " << filename;
			}
		}


		/************************************************************************/
		/* Read Functions													    */
		/************************************************************************/

		static void loadFromPLY(const std::string& filename, MeshData<FloatType>& mesh);

		static void loadFromOFF(const std::string& filename, MeshData<FloatType>& mesh);

		static void loadFromOBJ(const std::string& filename, MeshData<FloatType>& mesh);


		/************************************************************************/
		/* Write Functions													    */
		/************************************************************************/

		static void saveToPLY(const std::string& filename, const MeshData<FloatType>& mesh);

		static void saveToOFF(const std::string& filename, const MeshData<FloatType>& mesh);

		static void saveToOBJ(const std::string& filename, const MeshData<FloatType>& mesh);

	private:

#define OBJ_LINE_BUF_SIZE 256
		static void skipLine(char * buf, int size, FILE * fp)
		{
			//some weird files don't have newlines, which confused fgets
			while (1) {
				int c = fgetc(fp);
				if (c == EOF || c == '\n' || c == '\r') break;
			}
			//do {
			//	buf[size-1] = '$';
			//	fgets(buf, size, fp);
			//} while (buf[size-1] != '$');
		}
	};

	typedef MeshIO<float>	MeshIOf;
	typedef MeshIO<double>	MeshIOd;
}// end namespace
#include"MeshIO.cpp"
#endif  // CORE_MESH_MESHIO_H_
