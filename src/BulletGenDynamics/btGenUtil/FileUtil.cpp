#include "FileUtil.h"
#include <assert.h>
#include <cstdarg>
#include <experimental/filesystem>
#include <iostream>
#include <map>
#include <memory>
#include <string.h>
#include <sys/file.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

FILE *btFileUtil::OpenFile(const std::string &file_name, const char *mode)
{
    return OpenFile(file_name.c_str(), mode);
}

FILE *btFileUtil::OpenFile(const char *path, const char *mode)
{
    FILE *f = nullptr;
    f = fopen(path, mode);
    if (f == nullptr)
    {
        std::cout << "[error] Failed to open " << path << std::endl;
        exit(0);
        // assert(false); // failed to open file
    }
    return f;
}

void btFileUtil::CloseFile(FILE *&f)
{
    if (f != nullptr)
    {
        fclose(f);
        f = nullptr;
    }
}

void btFileUtil::ClearFile(const std::string &file_name)
{
    FILE *f = OpenFile(file_name, "w");
    CloseFile(f);
}

void btFileUtil::CreateFile(const std::string &file_name)
{
    ClearFile(file_name);
}

void btFileUtil::DeleteFile(const char *file_name)
{
    bool succc = remove(file_name) == 0;
    if (!succc)
    {
        printf("[error] btFileUtil::DeleteFile: Failed to delete %s!\n",
               file_name);
        assert(false);
    }
}

void btFileUtil::DeleteDir(const char *dir_name)
{
#ifndef __APPLE__
    std::cout << "delete a dir " << dir_name << " ";
    if (btFileUtil::ExistsDir(dir_name) == true &&
        std::experimental::filesystem::remove_all(dir_name))
    {
        std::cout << "succ\n";
    }
    else
        std::cout << "failed\n";
#endif
}

void btFileUtil::ClearDir(const char *dir_name)
{
#ifndef __APPLE__
    printf("Clear dir %s\n", std::string(dir_name).c_str());
    if (btFileUtil::ExistsDir(dir_name))
    {
        for (const auto &entry :
             std::experimental::filesystem::directory_iterator(dir_name))
        {
            btFileUtil::DeleteFile(entry.path());
        }
    }
#endif
}

void btFileUtil::CreateDir(const char *dir_name)
{
    // std::cout <<"create a dir " << dir_name << " ";
#ifndef __APPLE__
    if (btFileUtil::ExistsDir(dir_name) == false)
    {
        std::experimental::filesystem::create_directories(dir_name);
    }
#endif
}

std::string btFileUtil::RemoveExtension(const std::string &filename)
{
    size_t first_not_dot = filename.find_first_not_of('.');
    size_t last_dot = filename.find_last_of(".");
    if (last_dot == std::string::npos || last_dot <= first_not_dot)
    {
        return filename;
    }
    return filename.substr(0, last_dot);
}

void btFileUtil::DeleteFile(const std::string &filename)
{
    int err = remove(filename.c_str());
    if (err != 0)
    {
        printf("Failed to delete %s!\n", filename.c_str());
        assert(false);
    }
}

void btFileUtil::RenameFile(const std::string &ori_name,
                            const std::string &des_name)
{
    if (rename(ori_name.c_str(), des_name.c_str()) != 0)
    {
        std::cout << "[error] btFileUtil::RenameFile from " << ori_name
                  << " to " << des_name << "failed\n";
    }
}

void btFileUtil::CopyFile(const std::string &ori_name,
                          const std::string &des_name)
{
    if (btFileUtil::ExistsFile(ori_name) == false)
    {
        printf("[error] CopyFile: original file %s doesn't exist", ori_name.c_str());
        exit(1);
    }

    std::string target_dir_name = btFileUtil::GetDir(des_name);

    if (btFileUtil::ExistsDir(target_dir_name) == false)
    {
        printf("[warn] CopyFile: target dir %s doesn't exist, created now.",
               target_dir_name.c_str());

        btFileUtil::CreateDir(target_dir_name.c_str());
    }

    if (btFileUtil::ValidateFilePath(des_name) == false)
    {
        printf("[error] CopyFile: target file %s is invalid", des_name.c_str());
        exit(1);
    }

#ifndef __APPLE__
    if (false == std::experimental::filesystem::copy_file(ori_name, des_name))
    {
        printf("[error] CopyFile: from %s to %s failed", ori_name.c_str(), des_name.c_str());
        exit(1);
    }
#else
#include <fstream>

    std::ifstream src(ori_name);
    std::ofstream dst(des_name);
    if (src.fail() || dst.fail())
    {
        printf("[error] CopyFile: from %s to %s failed", ori_name.c_str(), des_name.c_str());
        exit(1);
    }
    dst << src.rdbuf();
#endif
    printf("[log] CopyFile: from %s to %s succ", ori_name.c_str(), des_name.c_str());
}

long int btFileUtil::GetFileSize(const std::string &filename)
{
    // returns size in bytes
    FILE *f = OpenFile(filename.c_str(), "rb");

    if (f != NULL)
    {
        fseek(f, 0, SEEK_END);
        long int f_size = ftell(f);
        CloseFile(f);
        return f_size;
    }
    return 0;
}

std::string btFileUtil::GetExtension(const std::string &filename)
{
    // remove leading '.'
    size_t dot_idx = 0;
    for (dot_idx; dot_idx < filename.size(); ++dot_idx)
    {
        if (filename[dot_idx] != '.')
        {
            break;
        }
    }

    std::string str = filename.substr(dot_idx);
    size_t pos = str.find_last_of(".");
    if (pos == std::string::npos)
    {
        return "";
    }
    return str.substr(pos + 1);
}

std::string btFileUtil::GetFilename(const std::string &path)
{
    int idx = 0;
    for (int i = static_cast<int>(path.size()) - 1; i >= 0; --i)
    {
        char curr_char = path[i];
        if (curr_char == '\\' || curr_char == '/')
        {
            idx = i + 1;
            break;
        }
    }

    std::string filename = path.substr(idx, path.size() - idx);
    return filename;
}

std::string btFileUtil::GetDir(const std::string &path)
{
    int idx = 0;
    for (int i = static_cast<int>(path.size()) - 1; i >= 0; --i)
    {
        char curr_char = path[i];
        if (curr_char == '\\' || curr_char == '/')
        {
            idx = i + 1;
            break;
        }
    }

    std::string dirname = path.substr(0, idx);
    return dirname;
}

void btFileUtil::FilterFilesByExtension(std::vector<std::string> &files,
                                        const std::string &ext)
{
    size_t i = 0;
    for (size_t j = 0; j < files.size(); ++j)
    {
        const std::string &curr_f = files[j];
        std::string curr_ext = GetExtension(curr_f);
        if (curr_ext == ext)
        {
            files[i] = curr_f;
            ++i;
        }
    }
    files.resize(i);
}

bool btFileUtil::ExistsFile(const std::string &file_name)
{
    FILE *f = nullptr;
    f = fopen(file_name.c_str(), "r");
    if (f != nullptr)
    {
        fclose(f);
        return true;
    }
    return false;
}

bool btFileUtil::ExistsDir(const std::string &dir_name)
{
    if (dir_name.size() == 0)
        return true;

    struct stat info;
    stat(dir_name.c_str(), &info);
    if (info.st_mode & S_IFDIR)
        return true;
    else
        return false;
}

bool btFileUtil::ValidateFilePath(const std::string &file_name)
{
    if (btFileUtil::ExistsFile(file_name) == true)
        return true;
    else
    {
        if (nullptr != btFileUtil::OpenFile(file_name, "w"))
        {
            btFileUtil::DeleteFile(file_name);
            return true;
        }
        else
            return false;
    }
}

std::string btFileUtil::ConcatFilename(const std::string &dir_,
                                       const std::string &file_)
{
    std::string final_name = "";
#ifndef __APPLE__
    std::experimental::filesystem::path dir(dir_), file(file_);
    std::experimental::filesystem::path full_path = dir / file;
    final_name = full_path.string();
#else
    if (dir_[dir_.size() - 1] != '/')
        final_name = dir_ + "/" + file_;
#endif

    return final_name;
}

void btFileUtil::FindLine(std::ifstream &f_stream, int line)
{
    f_stream.seekg(std::ios::beg);
    std::string str;
    int l = 0;
    while (std::getline(f_stream, str))
    {
        if (l == line - 1)
        {
            return;
        }
        ++l;
    }

    throw "Failed to find line in file stream\n";
}

std::string btFileUtil::ReadTextFile(const std::string &path)
{
    FILE *file = OpenFile(path.c_str(), "rb");
    std::string text = ReadTextFile(file);
    fclose(file);
    return text;
}
/*
bool btFileUtil::ReadArray(FILE* f, const std::string& tag_beg, const
std::string& tag_end, std::vector<double>& out_buffer)
{

        std::fstream f_stream("wft");
        out_buffer.clear();

        const char delims[] = " ,\t";

        std::string str;
        std::vector<char> char_array;

        bool succ = false;
        bool found = false;
        while (std::getline(f_stream, str))
        {
                if (str == tag_beg)
                {
                        found = true;
                }
                else if (str == tag_end)
                {
                        succ = found;
                        break;
                }
                else if (found)
                {
                        if (str.size() > 0)
                        {
                                char_array = std::vector<char>(str.begin(),
str.end()); char_array.push_back(0);

                                char* p_char = NULL;
                                p_char = strtok(&char_array[0], delims);

                                if (p_char != nullptr)
                                {
                                        std::string curr_tok(p_char);
                                        if (curr_tok.size() >= 2)
                                        {
                                                if (curr_tok[0] == '/' &&
curr_tok[1] == '/')
                                                {
                                                        continue;
                                                }
                                        }

                                        double val = std::atof(p_char);
                                        out_buffer.push_back(val);
                                }
                        }
                }
        }
        //f_stream.close();

        if (!succ)
        {
                out_buffer.clear();
        }
        return succ;
}
*/

bool btFileUtil::ReadTable(const std::string &filename,
                           std::vector<std::vector<double>> &out_buffer)
{
    std::fstream f_stream(filename);
    out_buffer.clear();

    const char delims[] = " ,\t";

    std::string str;
    std::vector<char> char_array;

    bool succ = true;
    while (std::getline(f_stream, str))
    {
        if (str.size() > 0)
        {
            std::vector<double> curr_array;

            char_array = std::vector<char>(str.begin(), str.end());
            char_array.push_back(0);

            char *p_char = NULL;
            p_char = strtok(&char_array[0], delims);

            while (p_char != nullptr)
            {
                std::string curr_tok(p_char);
                if (curr_tok.size() >= 2)
                {
                    if (curr_tok[0] == '/' && curr_tok[1] == '/')
                    {
                        break;
                    }
                }

                double val = std::atof(p_char);
                curr_array.push_back(val);
                p_char = strtok(NULL, delims);
            }

            if (curr_array.size() > 0)
            {
                out_buffer.push_back(curr_array);
            }
        }
    }

    if (!succ)
    {
        out_buffer.clear();
    }
    return succ;
}

bool btFileUtil::ReadMatrix(const std::string &filename,
                            Eigen::MatrixXd &out_mat)
{
    std::vector<std::vector<double>> data;
    btFileUtil::ReadTable(filename, data);

    bool succ = false;
    if (data.size() > 0 && data[0].size() > 0)
    {
        int n = static_cast<int>(data.size());
        int m = static_cast<int>(data[0].size());
        out_mat.resize(n, m);

        for (int i = 0; i < n; ++i)
        {
            auto curr_row = data[i];
            int curr_m = static_cast<int>(curr_row.size());
            assert(curr_m == m);
            for (int j = 0; j < m; ++j)
            {
                out_mat(i, j) = curr_row[j];
            }
        }
        succ = true;
    }
    return succ;
}

bool btFileUtil::WriteMatrix(const Eigen::MatrixXd &mat,
                             const std::string &out_filename)
{
    FILE *f = btFileUtil::OpenFile(out_filename, "w");
    bool succ = f != nullptr;

    if (succ)
    {
        for (int i = 0; i < mat.rows(); ++i)
        {
            for (int j = 0; j < mat.cols(); ++j)
            {
                if (j != 0)
                {
                    fprintf(f, ",");
                }
                fprintf(f, "%20.10f", mat(i, j));
            }
            fprintf(f, "\n");
        }
        btFileUtil::CloseFile(f);
    }
    return succ;
}

bool btFileUtil::AppendText(const std::string &str,
                            const std::string &out_filename)
{
    std::ofstream out_stream(out_filename, std::ios_base::app);

    bool succ = !out_stream.fail();
    if (succ)
    {
        out_stream << str;
    }

    out_stream.close();

    return succ;
}

std::string btFileUtil::GenerateSerialFilename(const std::string &root, int id)
{
    std::string path = btFileUtil::RemoveExtension(root);
    path =
        path + "_" + std::to_string(id) + "." + btFileUtil::GetExtension(root);
    return path;
}

std::string btFileUtil::GenerateRandomFilename(const std::string &root)
{
    std::string single_path = btFileUtil::RemoveExtension(root),
                final_path = "";
    do
    {
        final_path = single_path + "_" + std::to_string(std::rand()) + "." +
                     btFileUtil::GetExtension(root);
        // std::cout <<"try " << final_path << std::endl;
    } while (btFileUtil::ExistsFile(final_path) == true);
    return final_path;
}

std::string btFileUtil::ReadTextFile(FILE *f)
{
    if (!f)
    {
        return std::string("");
    }

    fseek(f, 0, SEEK_END);
    long size = ftell(f);
    fseek(f, 0, SEEK_SET);
    std::string text;
    std::unique_ptr<char> buffer(new char[size + 1]);

    buffer.get()[size] = 0;
    if (fread(buffer.get(), 1, size, f) == (unsigned long)size)
    {
        text = buffer.get();
    }

    buffer.reset();
    return text;
}

std::string btFileUtil::mFileLockDir = "./logs/controller_logs/locks/";
static std::map<std::string, int> write_descriptor;

#define LOCK_FAIL false
#define LOCK_SUCCESS true
bool btFileUtil::AddLock(const std::string &path)
{
    btFileUtil::CreateDir(mFileLockDir.c_str());
    std::string path_lock =
        mFileLockDir + btFileUtil::GetFilename(path) + ".lock";
    // open fail failed: add lock failed certainly
    int desc;
    if ((desc = open(path_lock.c_str(), O_RDWR | O_CREAT, 0644)) < 0)
    {
        return LOCK_FAIL;
    }
    write_descriptor[path_lock] = desc;
    // if "path" this file has been locked by other process, then this flock
    // will block this process untill it gets the lock. But if flock return <0,
    // some errors must occur.
    if (flock(desc, LOCK_EX) < 0)
    {
        close(desc);
        desc = write_descriptor[path_lock] = -1;
        return LOCK_FAIL;
    }
    return LOCK_SUCCESS;
}

bool btFileUtil::DeleteLock(const std::string &path)
{
    std::string path_lock =
        mFileLockDir + btFileUtil::GetFilename(path) + ".lock";
    int desc;
    // if the file descrimitor <0 which means this file hasn't been locked
    // correctly, leads to the failure of deleting lock
    if (write_descriptor.find(path_lock) == write_descriptor.end())
    {
        std::cout << "delete lock " << path_lock << " no descriptor\n";
        return LOCK_FAIL;
    }
    desc = write_descriptor[std::string(path_lock)];
    if (desc < 0)
    {
        std::cout << "delete lock " << path_lock << " descriptor < 0\n";
        return LOCK_FAIL;
    }
    if (flock(desc, LOCK_UN) < 0)
    {
        // nothing could be done here
    }
    close(desc);
    // btFileUtil::DeleteFile(path_lock);

    write_descriptor.erase(write_descriptor.find(path_lock));
    return LOCK_SUCCESS;
}