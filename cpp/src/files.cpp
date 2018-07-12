#include "files.h"


namespace IceHalo {
namespace Files {

bool exists(const char* filename)
{
    using namespace boost::filesystem;

    path p(filename);
    return exists(p);
}


File::File(const char* filename) : 
    filename(filename), file(nullptr), fileOpened(false),
    p(filename)
{ }


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

    file = std::fopen(filename.c_str(), modeBuffer);
    if (file) {
        fileOpened = true;
    } else {
        fileOpened = false;
    }
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