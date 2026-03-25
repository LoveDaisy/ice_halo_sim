// STB implementation file for LumiceGUI.
// Defines image read/write implementations used by .lmc file I/O.

#ifdef _WIN32
#define STBI_WINDOWS_UTF8
#define STBIW_WINDOWS_UTF8
#endif

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
