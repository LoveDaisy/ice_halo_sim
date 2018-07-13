#include "files.h"


namespace IceHalo {
namespace Files {

bool exists(const char* filename)
{
    using namespace boost::filesystem;

    path p(filename);
    return exists(p);
}


void listDataFiles(const char* dir, std::vector<File>& files)
{
    using namespace boost::filesystem;

    path p(dir);
    for (directory_entry& x : directory_iterator(p)) {
        if (x.path().extension() == ".bin") {
            files.emplace_back(x.path().c_str());
        }
    }
}


std::string pathJoin(const std::string& p1, const std::string& p2)
{
    using namespace boost::filesystem;

    path p(p1);
    p /= (p2);
    return p.string();
}


File::File(const char* filename) : 
    file(nullptr), fileOpened(false),
    p(filename)
{ }


File::File(const char* path, const char* filename) :
    file(nullptr), fileOpened(false),
    p(path)
{
    p /= filename;
}


File::~File()
{
    if (fileOpened) {
        fclose(file);
    }
}


bool File::open(uint8_t mode)
{
    using namespace boost::filesystem;

    if (!exists(p.parent_path())) {
        create_directories(p.parent_path());
    }

    char modeBuffer[32];
    const char *m1;
    if (mode & OpenMode::READ) {
        m1 = "r";
    } else if (mode & OpenMode::WRITE) {
        m1 = "w";
    } else if (mode & OpenMode::APPEND) {
        m1 = "a";
    }
    const char *m2 = (mode & OpenMode::BINARY) ? "b" : "";
    sprintf(modeBuffer, "%s%s", m1, m2);

    file = std::fopen(p.c_str(), modeBuffer);
    fileOpened = file != nullptr;
    return fileOpened;
}


bool File::close()
{
    if (fileOpened) {
        std::fclose(file);
        file = nullptr;
    }
    return true;
}


size_t File::getSize()
{
    using namespace boost::filesystem;
    
    auto size = file_size(p);
    if (size == static_cast<uintmax_t>(-1)) {
        return 0;
    } else {
        return static_cast<size_t>(size);
    }
}


}
}