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

#define instance() (Module::getInstance<graphics::Graphics>(Module::M_GRAPHICS))

static int luax_checkgraphicscreated(lua_State *L)
{
	if (!instance()->isCreated())
		return luaL_error(L, "love.graphics cannot function without a window.");
	return 0;
}

// MEG2.SHADER

static int w_getShaderSource(lua_State *L, int startidx, std::vector<std::string> &stages, graphics::Shader::CompileOptions &options)
{
	using namespace love::filesystem;

	luax_checkgraphicscreated(L);

	auto fs = Module::getInstance<Filesystem>(Module::M_FILESYSTEM);

	// read any filepath arguments
	for (int i = startidx; i < startidx + 2; i++)
	{
		if (!lua_isstring(L, i))
		{
			if (luax_cangetfiledata(L, i))
			{
				FileData *fd = luax_getfiledata(L, i);

				lua_pushlstring(L, (const char *) fd->getData(), fd->getSize());
				fd->release();

				lua_replace(L, i);
			}
			continue;
		}

		size_t slen = 0;
		const char *str = lua_tolstring(L, i, &slen);

		Filesystem::Info info = {};
		if (fs != nullptr && fs->getInfo(str, info))
		{
			FileData *fd = nullptr;
			luax_catchexcept(L, [&](){ fd = fs->read(str); });

			lua_pushlstring(L, (const char *) fd->getData(), fd->getSize());
			fd->release();

			lua_replace(L, i);
		}
		else
		{
			// Check if the argument looks like a filepath - we want a nicer
			// error for misspelled filepath arguments.
			if (slen > 0 && slen < 64 && !strchr(str, '\n'))
			{
				const char *ext = strchr(str, '.');
				if (ext != nullptr && !strchr(ext, ';') && !strchr(ext, ' '))
					return luaL_error(L, "Could not open file %s. Does not exist.", str);
			}
		}
	}

	bool has_arg1 = lua_isstring(L, startidx + 0) != 0;
	bool has_arg2 = lua_isstring(L, startidx + 1) != 0;

	// require at least one string argument
	if (!(has_arg1 || has_arg2))
		luaL_checkstring(L, startidx);

	if (has_arg1)
		stages.push_back(luax_checkstring(L, startidx + 0));
	if (has_arg2)
		stages.push_back(luax_checkstring(L, startidx + 1));

	int optionsidx = has_arg2 ? startidx + 2 : startidx + 1;
	if (!lua_isnoneornil(L, optionsidx))
	{
		luaL_checktype(L, optionsidx, LUA_TTABLE);
		lua_getfield(L, optionsidx, "defines");
		if (!lua_isnoneornil(L, -1))
		{
			if (!lua_istable(L, -1))
				luaL_argerror(L, optionsidx, "expected 'defines' field to be a table");

			lua_pushnil(L);
			while (lua_next(L, -2))
			{
				std::string defname;
				std::string defval;

				if (lua_type(L, -2) == LUA_TNUMBER && lua_type(L, -1) == LUA_TSTRING)
					defname = luaL_checkstring(L, -1);
				else if (lua_type(L, -2) != LUA_TSTRING)
					luaL_argerror(L, optionsidx, "all fields in the 'defines' table must use string keys.");
				else
				{
					defname = luaL_checkstring(L, -2);
					if (lua_type(L, -1) == LUA_TBOOLEAN)
						defval = luax_toboolean(L, -1) ? "1" : "0";
					else
					{
						const char *val = lua_tostring(L, -1);
						if (val == nullptr)
							return luaL_argerror(L, optionsidx, "'defines' table values must be strings, numbers, or booleans.");
						defval = val;
					}
				}

				options.defines[defname] = defval;

				lua_pop(L, 1);
			}
		}
		lua_pop(L, 1);

		lua_getfield(L, optionsidx, "debugname");
		if (!lua_isnoneornil(L, -1))
			options.debugName = luax_checkstring(L, -1);
		lua_pop(L, 1);

		for (int feature = 0; feature < graphics::Shader::FEATURE_MAX_ENUM; ++feature)
		{
			const char *featureKey;
			if (graphics::Shader::getConstant((graphics::Shader::Feature)feature, featureKey))
			{
				lua_getfield(L, optionsidx, featureKey);
				if (!lua_isnoneornil(L, -1))
				{
					options.features[feature] = lua_toboolean(L, -1);
				}
				lua_pop(L, 1);
			}
		}
	}

	return 0;
}

int w_validateShader(lua_State *L)
{
	bool gles = luax_checkboolean(L, 1);

	std::vector<std::string> stages;
	graphics::Shader::CompileOptions options;
	w_getShaderSource(L, 2, stages, options);

	bool success = true;
	std::string err;
	try
	{
		success = instance()->validateShader(gles, stages, options, err);
	}
	catch (love::Exception &e)
	{
		success = false;
		err = e.what();
	}

	luax_pushboolean(L, success);

	if (!success)
	{
		luax_pushstring(L, err);
		return 2;
	}

	return 1;
}

int w_newShader(lua_State *L)
{
	std::vector<std::string> stages;
	graphics::Shader::CompileOptions options;
	w_getShaderSource(L, 1, stages, options);

	bool should_error = false;
	try
	{
		graphics::Shader *shader = instance()->newShader(stages, options);
		if (shader->isUsingDeprecatedTextureFunctions())
			luax_markdeprecated(L, 1, "texture2D() or texture3D() or textureCube() function calls in shader code", API_CUSTOM, DEPRECATED_REPLACED, "texture() function calls");
		if (shader->isUsingDeprecatedTextureUniform())
			luax_markdeprecated(L, 1, "'texture' uniform variable name in shader code", API_CUSTOM, DEPRECATED_NO_REPLACEMENT, "");
		if (!shader->getUnsetVertexInputLocationsString().empty())
		{
			std::string str = "vertex input attribute(s) " + shader->getUnsetVertexInputLocationsString() + " without a 'location' layout qualifier in shader code";
			luax_markdeprecated(L, 1, str.c_str(), API_CUSTOM, DEPRECATED_REPLACED, "layout(location = #) qualifier for vertex inputs");
		}
		luax_pushtype(L, shader);
		shader->release();
	}
	catch (love::Exception &e)
	{
		luax_getfunction(L, "graphics", "_transformGLSLErrorMessages");
		lua_pushstring(L, e.what());

		// Function pushes the new error string onto the stack.
		lua_pcall(L, 1, 1, 0);
		should_error = true;
	}

	if (should_error)
		return lua_error(L);

	return 1;
}

static int w_shader_load(lua_State *L)
{
    std::vector<std::string> stages;
    graphics::Shader::CompileOptions options;
    w_getShaderSource(L, 1, stages, options);

    try
    {
        graphics::Shader *shader = instance()->newShader(stages, options);

        luax_pushtype(L, shader);
        shader->release();

        return 1;
    }
    catch (love::Exception &e)
    {
        lua_pushnil(L);
        luax_pushstring(L, e.what());
        return 2;
    }
}

void setShaderFunc(lua_State *L){
		
}

// MEG2.RENDER

static int w_render_finish(lua_State *L)
{
	instance()->setShader();
	return 0;
}

static void setRenderFunc(lua_State *L)
{
	lua_newtable(L);
	lua_pushcfunction(L, w_render_finish);
	lua_setfield(L, -2, "finish");
}

extern "C" int luaopen_love_meg2(lua_State *L)
{
	lua_newtable(L); // love.meg2

	love::meg2::setRenderFunc(L);
	lua_setfield(L, -2, "render");

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