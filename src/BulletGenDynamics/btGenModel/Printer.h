#ifndef ROBOT_UTILS_PRINTERR_H
#define ROBOT_UTILS_PRINTERR_H
#include <iostream>
#include <vector>
#include "ModelEigenUtils.h"
#include <fstream>
#include <cstdarg>
#include <algorithm>

class Printer {
public:

	static void ScreenLog(const char* str) {
		std::cout << "[Log] " << str << std::endl;
	}

	static void ScreenLog(int d, const char* str) {
		std::cout << "[Log] " << str << ": " << d << std::endl;
	}

	static void ScreenLog(const char * str, const char * str2){
		std::cout << "[Log] " << str << ": " << str2 << std::endl;
	}

	static void Error(const char* error) {
		std::cerr << "[ERROR] " << error << std::endl;
	}

	static void FormatScreenLog(const char * fmt, ...)
	{
		va_list ap;
		va_start(ap, fmt);
    	int real_size = vsnprintf(buf, buf_size, fmt, ap);
    
    	const std::string & log = std::string(buf, std::min(real_size, static_cast<int>(buf_size)));
		Printer::ScreenLog(log.c_str());
	}

	static void FormatError(const char * fmt, ...)
	{
		va_list ap;
		va_start(ap, fmt);
    	int real_size = vsnprintf(buf, buf_size, fmt, ap);
    
    	const std::string & log = std::string(buf, std::min(real_size, static_cast<int>(buf_size)));
		Printer::Error(log.c_str());
	}

	static void Print(const std::vector<int>& data, const char* prefix) {
		std::cout << prefix << ": \n";
		for (unsigned int i = 0; i < data.size(); i++) {
			std::cout << i << ": " << data[i] << std::endl;
		}
	}

	static void Print(const std::vector<double>& data, const char* prefix) {
		std::cout << prefix << ": \n";
		for (unsigned int i = 0; i < data.size(); i++) {
			std::cout << i << ": " << data[i] << std::endl;
		}
	}

	static void Print(const EIGEN_V_tVector3f& data, const char* prefix) {
		std::cout << prefix << ": \n";
		for (unsigned int i = 0; i < data.size(); i++) {
			std::cout << i << ": " << data[i][0] << ", " << data[i][1] << ", " << data[i][2] << std::endl;
		}
	}
	
	static void Print(const EIGEN_V_tVector3d& data, const char* prefix) {
		std::cout << prefix << ": \n";
		for (unsigned int i = 0; i < data.size(); i++) {
			std::cout << i << ": " << data[i][0] << ", " << data[i][1] << ", " << data[i][2] << std::endl;
		}
	}

	static void Print(const tVector3f& data, const char* prefix) {
		std::cout << prefix << ": \n";
		std::cout << data[0] << ", " << data[1] << ", " << data[2] << std::endl;
	}

	static void Print(const tVector3d& data, const char* prefix) {
		std::cout << prefix << ": \n";
		std::cout << data[0] << ", " << data[1] << ", " << data[2] << std::endl;
	}

	static void Print(const tVector3f& data) {
		std::cout << data[0] << ", " << data[1] << ", " << data[2] << std::endl;
	}

	static void Print(const Eigen::Vector4f& data) {
		std::cout << data[0] << ", " << data[1] << ", " << data[2] << data[3] << std::endl;
	}

	static void Print(const tVector3d& data) {
		std::cout << data[0] << ", " << data[1] << ", " << data[2] << std::endl;
	}

	static void Print(const tVector& data) {
		std::cout << data[0] << ", " << data[1] << ", " << data[2] << data[3] << std::endl;
	}

	static void Print(const Eigen::Matrix4f& m, const char* name) {
		std::cout << name << ":\n" << m << std::endl;
	}

	static void Print(const tMatrix& m, const char* name) {
		std::cout << name << ":\n" << m << std::endl;
	}

	static void Log(const tVectorXd& data, int group_num, const char* file_path) {
		std::fstream fout;
		fout.open(file_path, std::ios::out);
		for (unsigned int i = 0; i < data.size(); i++) {
			fout << data[i] << ", ";
			if (i % group_num == 0) fout << "\n";
		}
		fout.close();
	}

	static void Log(const std::vector<double>& data, int group_num, const char* prefix, const char *file_path) {
		std::fstream fout;
		fout.precision(20);
		fout.open(file_path, std::ios::out);
		const int n = static_cast<int>(data.size()) / group_num;
		for ( int i = 0; i < n; i++) {
			fout << i << std::endl;
			for( int j = 0; j < group_num; j ++ ) {
				fout << j << ", " << data[i*group_num+j] << std::endl;
			}
			fout << "=========\n";
		}
		fout.close();
	}

	static void Log(const tVectorXd& data, int group_num, const char* prefix, const char* file_path) {
		std::fstream fout;
		fout.open(file_path, std::ios::out);
		fout.precision(20);
		const int n = static_cast<int>(data.size()) / group_num;
		for (int i = 0; i < n; i++) {
			fout << i << std::endl;
			for (int j = 0; j < group_num; j++) {
				fout << j << ", " << data[i * group_num + j] << std::endl;
			}
			fout << "=========\n";
		}
		fout.close();
	}

	static void LogCSV(const tVectorXd& data, int group_num, const char* prefix, const char* file_path) {
		std::fstream fout;
		fout.open(file_path, std::ios::out);
		fout.precision(20);
		const int n = static_cast<int>(data.size()) / group_num;
		for (int i = 0; i < n; i++) {
			for (int j = 0; j < group_num; j++) {
				fout << j << ", " << data[i * group_num + j];
			}
		}
		fout.close();
	}

	static void LogCSV(const std::vector<double> &data, int group_num, const char* prefix, const char* file_path) {
		std::fstream fout;
		fout.open(file_path, std::ios::out);
		fout.precision(20);
		const int n = static_cast<int>(data.size()) / group_num;
		for (int i = 0; i < n; i++) {
			for (int j = 0; j < group_num; j++) {
				fout << j << ", " << data[i * group_num + j];
			}
		}
		fout.close();
	}

	//static void Log(std::vector<double>& data, int group_num, const char* prefix, const char* file_path) {
	//	std::fstream fout;
	//	fout.open(file_path, std::ios::out);
	//	const int n = static_cast<int>(data.size()) / group_num;
	//	for (int i = 0; i < n; i++) {
	//		fout << i << std::endl;
	//		for (int j = 0; j < group_num; j++) {
	//			fout << j << ", " << data[i * group_num + j] << std::endl;
	//		}
	//		fout << "=========\n";
	//	}
	//	fout.close();
	//}
	static void Log(const std::vector<std::vector<double>>& data, int group_num, const char* prefix, const char* file_path) {
		if (data.empty()) return;
		std::fstream fout;
		fout.open(file_path, std::ios::out);
		fout << prefix << '\n';
		for(size_t k = 0; k < data.size(); ++ k) {
			fout << "itr: " << k << std::endl;
			const int n = static_cast<int>(data[0].size()) / group_num;
			for (int i = 0; i < n; i++) {
				fout << i << std::endl;
				for (int j = 0; j < group_num; j++) {
					fout << j << ", " << data[k][i * group_num + j] << std::endl;
				}
				fout << "=========\n";
			}
		}
		fout.close();
	}

	static void Log(const EIGEN_V_tVector3f&data, const char* prefix, const char * file_path) {
		std::fstream fout;
		fout.open(file_path, std::ios::out);
		for (unsigned int i = 0; i < data.size(); i++) {
			fout << i << ": " << data[i][0] << ", " << data[i][1] << ", " << data[i][2] << std::endl;
		}
		fout.close();
	}

	static void Log(const EIGEN_V_tVector3d& data, const char* prefix, const char* file_path) {
		std::fstream fout;
		fout.open(file_path, std::ios::out);
		for (unsigned int i = 0; i < data.size(); i++) {
			fout << i << ": " << data[i][0] << ", " << data[i][1] << ", " << data[i][2] << std::endl;
		}
		fout.close();
	}

	static void Log(const EIGEN_V_tVector3d& data, int group_num, const char* prefix, const char* file_path) {
		std::fstream fout;
		const int n = static_cast<int>(data.size()) / group_num;

		fout.open(file_path, std::ios::out);
		int st = 0;
		for (int i = 0; i < n; i++) {
			fout << i << ": \n";
			for (int j = 0; j < group_num; ++j) 
				fout << data[st + j][0] << ", " << data[st + j][1] << ", " << data[st + j][2] << std::endl;
			st += group_num;
		}
		fout.close();
	}

	static void Log(const EIGEN_V_tVector3d& data, int group_num, const char* prefix, const char* file_path, bool norm) {
		std::fstream fout;
		const int n = static_cast<int>(data.size()) / group_num;

		fout.open(file_path, std::ios::out);
		int st = 0;
		for (int i = 0; i < n; i++) {
			fout << i << ": \n";
			for (int j = 0; j < group_num; ++j) {
				fout << data[st + j][0] << ", " << data[st + j][1] << ", " << data[st + j][2];
				if (norm) fout << " norm: " << data[st + j].norm();
				fout << "\n";
			}
				
				
			st += group_num;
		}
		fout.close();
	}

	static void Log(const EIGEN_V_tVector& data, int group_num, const char* prefix, const char* file_path) {
		std::fstream fout;
		fout.precision(8);
		const int n = static_cast<int>(data.size()) / group_num;

		fout.open(file_path, std::ios::out);
		int st = 0;
		for (int i = 0; i < n; i++) {
			fout << i << ": \n";
			for (int j = 0; j < group_num; ++j)
				fout << data[st + j][0] << ", " << data[st + j][1] << ", " << data[st + j][2] << ", " << data[st + j][3] << std::endl;
			st += group_num;
		}
		fout.close();
	}

	static void Log(const tMatrix& m, const char* name, const char* file_path) {
		std::fstream fout;
		fout.open(file_path, std::ios::app);
		fout << name << ":\n" << m << std::endl;
		fout.close();
	}

	static void Log(const tMatrixXd& m, const char* file_path) {
		std::fstream fout;
		fout.open(file_path, std::ios::out);
		auto w = m.cols();
		auto h = m.rows();

		for (int i = 0; i < w; i++) {
			fout << i << ": ";
			for (int j = 0; j < h; j++) {
				fout << m.data()[i * h + j] << ", ";
			}
			fout << "\n";
		}
		fout.close();
	}

	static void Log(const tMatrixXd& m, std::vector<int>& col_id, const char* file_path) {
		std::fstream fout;
		fout.open(file_path, std::ios::out);
		const auto w = m.cols();
		const auto h = m.rows();
		for(size_t i = 0; i < col_id.size(); i++) {
			fout << col_id[i] << ": ";
			for ( int j = 0; j < h; j ++) {
				fout << m.col(col_id[i])[j] << ", ";
			}
			fout << " \n";
		}
		fout.close();
	}
	static void Log(const EIGEN_V_MATXD& ms, const char* file_path) {
		std::fstream fout;
		fout.open(file_path, std::ios::app);
		fout.precision(20);
		for( size_t i = 0; i < ms.size(); i ++) {
			fout << i << ": \n";
			fout << ms[i] << std::endl;
			//const auto w = ms[i].cols();
			//const auto h = ms[i].rows();
			//for (int j = 0; j < h; j++) {
			//	fout << j << ": ";
			//	for (int k = 0; k < w; k++) {
			//		fout << ms[i].row(j).data()[k] << ", ";
			//	}
			//	fout << " \n";
			//}
		}
		fout.close();
	}

	static void Log(const EIGEN_VV_MATXD& ms, const char* file_path) {
		std::fstream fout;
		fout.open(file_path, std::ios::out);
		int itr = 0;
		for( auto& m: ms) {
			fout << "itr: " << itr++ << "\n";;
			for (size_t i = 0; i < m.size(); i++) {
				fout << i << ": \n";

				const auto w = m[i].cols();
				const auto h = m[i].rows();
				for (int j = 0; j < w; j++) {
					fout << j << ": ";
					for (int k = 0; k < h; k++) {
						fout << m[i].col(j)[k] << ", ";
					}
					fout << " \n";
				}
			}
		}
		
		fout.close();
	}


	static void Log(const EIGEN_VV_MATXD& ms, std::vector<std::string> names, const char* file_path) {
		std::fstream fout;
		fout.open(file_path, std::ios::out);
		int itr = 0;
		for (auto& m : ms) {
			fout << "itr: " << itr++ << "\n";;
			for (size_t i = 0; i < m.size(); i++) {
				fout << names[i] << ": \n";

				const auto w = m[i].cols();
				const auto h = m[i].rows();
				for (int j = 0; j < w; j++) {
					//fout << j << ": ";
					for (int k = 0; k < h; k++) {
						fout << m[i].row(j)[k] << ", ";
					}
					fout << " \n";
				}
			}
		}

		fout.close();
	}

	static void Log(const std::vector<const std::vector<double>*>& data, int group_num, const char* prefix, const char* file_path) {
		std::fstream fout;
		fout.open(file_path, std::ios::out);
		fout << prefix << '\n';
		for (size_t k = 0; k < data.size(); ++k) {
			fout << data[k] << std::endl;
		}
		fout.close();
	}

	static void Log(const EIGEN_VV_tVector3d& data, std::vector<std::string> names, const char* file_path) {
		std::fstream fout;
		fout.open(file_path, std::ios::out);
		for (size_t i = 0; i < data.size(); ++i) {
			fout << i << std::endl;
			for (size_t j = 0; j < data[i].size(); ++j) {
				fout << names[j] << ": " << data[i][j][0] << ", " << data[i][j][1] << ", " << data[i][j][2] << "\n";
			}
 			fout << "===\n" << std::endl;
		}
		fout.close();
	}

	static void Log(const EIGEN_V_VECXD& data, const char* prefix, const char* file_path) {
		std::fstream fout;
		fout.precision(10);
		fout.open(file_path, std::ios::out);
		fout << prefix << ":\n";
		for (size_t i = 0; i < data.size(); ++i) {
			fout << i << ": ";
			fout << data[i].transpose();
			fout << "\n";
		}
		fout.close();
	}

	static bool AskForUserConfirmation(const std::string & question);

private:
	inline const static size_t buf_size = 1000;
    inline static char buf[buf_size];
};

 
#endif