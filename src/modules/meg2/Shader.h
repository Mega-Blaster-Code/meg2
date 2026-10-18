#pragma once

#include "common/runtime.h"
#include "common/Module.h"

#include "modules/graphics/Graphics.h"
#include "modules/graphics/Shader.h"
#include "modules/graphics/wrap_Shader.h"
#include "filesystem/Filesystem.h"
#include "filesystem/wrap_Filesystem.h"



namespace love
{
namespace meg2
{

void MEG2_setShaderFunc(lua_State *L);

void MEG2_pushDefaultShader(lua_State *L);

} // meg2
} // love