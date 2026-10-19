#include "common/runtime.h"
#include "common/Module.h"

#include "modules/graphics/Graphics.h"
#include "modules/graphics/Shader.h"
#include "modules/graphics/wrap_Shader.h"
#include "filesystem/Filesystem.h"
#include "filesystem/wrap_Filesystem.h"

#include "modules/mgold/Shader.h"
#include "modules/mgold/Render.h"
#include "modules/mgold/Textures.h"
#include "modules/mgold/Matrices.h"

namespace love
{
namespace mgold
{

extern "C" int luaopen_love_mgold(lua_State *L)
{
	lua_newtable(L); // love.mgold

	love::mgold::MGOLD_setMatricesFunc(L);
	lua_setfield(L, -2, "matrices");

	printf("Created Matrices\n");

	printf("Opened mgold\n");
	love::mgold::MGOLD_setShaderFunc(L);
	lua_setfield(L, -2, "shader");

	printf("Created Shaders\n");

	love::mgold::MGOLD_setRenderFunc(L);
	lua_setfield(L, -2, "render");

	printf("Created Render\n");

	love::mgold::MGOLD_setTextureFunc(L);
	lua_setfield(L, -2, "textures");

	printf("Created Textures\n");

	lua_getglobal(L, "love");
	if (lua_istable(L, -1))
	{
		lua_pushvalue(L, -2);
		lua_setfield(L, -2, "mgold");
	}
	lua_pop(L, 1);

	return 1;
}
 
} // mgold
} // love