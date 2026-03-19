/*****************************************************************************************
**          Copyright(c) 2015-2026 MicroStrain by HBK. All rights reserved.             **
**                                                                                      **
**    MIT Licensed. See the included LICENSE file for a copy of the full MIT License.   **
*****************************************************************************************/

#pragma once

#include "mscl/Version.h"

#ifndef SWIG
#define MSCL_MAJOR 68
#define MSCL_MINOR 1
#define MSCL_PATCH 0

#define MSCL_GIT_COMMIT "c1ca61e6-dirty"
#endif // !SWIG

namespace mscl
{
    //API Variable: MSCL_VERSION
    //  Gets the <Version> of MSCL.
    static const Version MSCL_VERSION = Version(MSCL_MAJOR, MSCL_MINOR, MSCL_PATCH);
} // namespace mscl
