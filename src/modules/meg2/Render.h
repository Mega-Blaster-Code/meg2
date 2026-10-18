#pragma once

#include "common/runtime.h"
#include "common/Module.h"

#include "modules/graphics/Graphics.h"
#include "modules/graphics/Shader.h"
#include "modules/graphics/wrap_Shader.h"
#include "filesystem/Filesystem.h"
#include "filesystem/wrap_Filesystem.h"

#include "modules/meg2/Shader.h"


namespace love
{
namespace meg2
{

void MEG2_setRenderFunc(lua_State *L);

}
}