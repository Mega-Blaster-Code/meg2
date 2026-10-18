#include "common/runtime.h"
#include "common/Module.h"

#include "modules/graphics/Graphics.h"
#include "modules/graphics/Shader.h"
#include "modules/graphics/wrap_Shader.h"
#include "filesystem/Filesystem.h"
#include "filesystem/wrap_Filesystem.h"

#include "modules/meg2/Shader.h"
#include "modules/meg2/Render.h"

namespace love
{
namespace meg2
{

extern "C" int luaopen_love_meg2(lua_State *L)
{
	lua_newtable(L); // love.meg2

	printf("Opened meg2\n");
	love::meg2::MEG2_setShaderFunc(L);
	lua_setfield(L, -2, "shader");

	printf("Created Shaders\n");

	love::meg2::MEG2_setRenderFunc(L);
	lua_setfield(L, -2, "render");

	printf("Created Render\n");

	lua_getglobal(L, "love");
	if (lua_istable(L, -1))
	{
		lua_pushvalue(L, -2);
		lua_setfield(L, -2, "meg2");
	}
	lua_pop(L, 1);

	return 1;
}
 
} // meg2
} // love