#include "modules/mgold/Matrices.h"

#include <cmath>
#include <cstdio>

namespace love
{

namespace mgold
{

static void m_set(lua_State *L, int idx, int i, double v){
	lua_pushnumber(L, v);
	lua_rawseti(L, idx, i);
}

static double m_get(lua_State *L, int idx, int i){
	lua_rawgeti(L, idx, i);
	double v = lua_tonumber(L, -1);
	lua_pop(L, 1);
	return v;
}

static void checkVec3(lua_State *L, int idx, double *out){
	luaL_checktype(L, idx, LUA_TTABLE);
	for (int i = 0; i < 3; i++){
		lua_rawgeti(L, idx, i + 1);
		if (!lua_isnumber(L, -1)){
			luaL_error(L, "argument #%d: expected table with 3 numbers", idx);
		}
		out[i] = lua_tonumber(L, -1);
		lua_pop(L, 1);
	}
}

static void vecNormalize(double &x, double &y, double &z){
	double len = std::sqrt(x * x + y * y + z * z);
	if (len == 0.0){
		x = y = z = 0.0;
		return;
	}
	x /= len; y /= len; z /= len;
}

static void vecCross(double ax, double ay, double az, double bx, double by, double bz, double &rx, double &ry, double &rz){
	rx = ay * bz - az * by;
	ry = az * bx - ax * bz;
	rz = ax * by - ay * bx;
}

static double vecDot(double ax, double ay, double az, double bx, double by, double bz){
	return ax * bx + ay * by + az * bz;
}

static int w_newMatrix(lua_State *L){
	lua_createtable(L, 16, 0);
	int t = lua_gettop(L);

	static const double identity[16] = {
		1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1
	};

	for (int i = 0; i < 16; i++){
		m_set(L, t, i + 1, identity[i]);
	}

	lua_pushvalue(L, lua_upvalueindex(1));
	lua_setmetatable(L, t);

	return 1;
}

static int w_tostring(lua_State *L){
	luaL_checktype(L, 1, LUA_TTABLE);

	char buf[512];
	double v[16];
	for (int i = 0; i < 16; i++){
		v[i] = m_get(L, 1, i + 1);
	}

	snprintf(buf, sizeof(buf),
		"%f\t%f\t%f\t%f\n%f\t%f\t%f\t%f\n%f\t%f\t%f\t%f\n%f\t%f\t%f\t%f",
		v[0], v[1], v[2], v[3],
		v[4], v[5], v[6], v[7],
		v[8], v[9], v[10], v[11],
		v[12], v[13], v[14], v[15]);

	lua_pushstring(L, buf);
	return 1;
}

static int w_setTransformationMatrix(lua_State *L){
	luaL_checktype(L, 1, LUA_TTABLE);

	double t[3], r[3], s[3];
	checkVec3(L, 2, t);
	checkVec3(L, 3, r);
	checkVec3(L, 4, s);

	m_set(L, 1, 4, t[0]);
	m_set(L, 1, 8, t[1]);
	m_set(L, 1, 12, t[2]);

	double sx = s[0], sy = s[1], sz = s[2];
	double rx = r[0], ry = r[1], rz = r[2];

	if (rx == 0 && ry == 0 && rz == 0){
		m_set(L, 1, 1, sx); m_set(L, 1, 2, 0);  m_set(L, 1, 3, 0);
		m_set(L, 1, 5, 0);  m_set(L, 1, 6, sy); m_set(L, 1, 7, 0);
		m_set(L, 1, 9, 0);  m_set(L, 1, 10, 0);  m_set(L, 1, 11, sz);
	}
	else{
		double ca = std::cos(rz), cb = std::cos(ry), cc = std::cos(rx);
		double sa = std::sin(rz), sb = std::sin(ry), sc = std::sin(rx);

		m_set(L, 1, 1, ca * cb * sx);
		m_set(L, 1, 2, (ca * sb * sc - sa * cc) * sy);
		m_set(L, 1, 3, (ca * sb * cc + sa * sc) * sz);

		m_set(L, 1, 5, sa * cb * sx);
		m_set(L, 1, 6, (sa * sb * sc + ca * cc) * sy);
		m_set(L, 1, 7, (sa * sb * cc - ca * sc) * sz);

		m_set(L, 1, 9, -sb * sx);
		m_set(L, 1, 10, cb * sc * sy);
		m_set(L, 1, 11, cb * cc * sz);
	}

	m_set(L, 1, 13, 0);
	m_set(L, 1, 14, 0);
	m_set(L, 1, 15, 0);
	m_set(L, 1, 16, 1);

	return 0;
}

static int w_getScale(lua_State *L){
	luaL_checktype(L, 1, LUA_TTABLE);

	double m1 = m_get(L, 1, 1), m5 = m_get(L, 1, 5), m9 = m_get(L, 1, 9);
	double m2 = m_get(L, 1, 2), m6 = m_get(L, 1, 6), m10 = m_get(L, 1, 10);
	double m3 = m_get(L, 1, 3), m7 = m_get(L, 1, 7), m11 = m_get(L, 1, 11);

	lua_pushnumber(L, std::sqrt(m1 * m1 + m5 * m5 + m9 * m9));
	lua_pushnumber(L, std::sqrt(m2 * m2 + m6 * m6 + m10 * m10));
	lua_pushnumber(L, std::sqrt(m3 * m3 + m7 * m7 + m11 * m11));

	return 3;
}

static int w_lookAtFrom(lua_State *L){
	luaL_checktype(L, 1, LUA_TTABLE);

	double pos[3], target[3], up[3];
	checkVec3(L, 2, pos);
	checkVec3(L, 3, target);
	checkVec3(L, 4, up);

	m_set(L, 1, 4, pos[0]);
	m_set(L, 1, 8, pos[1]);
	m_set(L, 1, 12, pos[2]);

	double sx, sy, sz;
	if (!lua_isnoneornil(L, 5)){
		double sc[3];
		checkVec3(L, 5, sc);
		sx = sc[0]; sy = sc[1]; sz = sc[2];
	}
	else{
		double m1 = m_get(L, 1, 1), m5 = m_get(L, 1, 5), m9 = m_get(L, 1, 9);
		double m2 = m_get(L, 1, 2), m6 = m_get(L, 1, 6), m10 = m_get(L, 1, 10);
		double m3 = m_get(L, 1, 3), m7 = m_get(L, 1, 7), m11 = m_get(L, 1, 11);
		sx = std::sqrt(m1 * m1 + m5 * m5 + m9 * m9);
		sy = std::sqrt(m2 * m2 + m6 * m6 + m10 * m10);
		sz = std::sqrt(m3 * m3 + m7 * m7 + m11 * m11);
	}

	double f_x = pos[0] - target[0], f_y = pos[1] - target[1], f_z = pos[2] - target[2];
	vecNormalize(f_x, f_y, f_z);

	double s_x, s_y, s_z;
	vecCross(up[0], up[1], up[2], f_x, f_y, f_z, s_x, s_y, s_z);
	vecNormalize(s_x, s_y, s_z);

	double u_x, u_y, u_z;
	vecCross(f_x, f_y, f_z, s_x, s_y, s_z, u_x, u_y, u_z);

	m_set(L, 1, 1, f_x * sx); m_set(L, 1, 2, s_x * sy); m_set(L, 1, 3, u_x * sz);
	m_set(L, 1, 5, f_y * sx); m_set(L, 1, 6, s_y * sy); m_set(L, 1, 7, u_y * sz);
	m_set(L, 1, 9, f_z * sx); m_set(L, 1, 10, s_z * sy); m_set(L, 1, 11, u_z * sz);

	return 0;
}

static int w_setProjectionMatrix(lua_State *L)
{
	luaL_checktype(L, 1, LUA_TTABLE);

	double fov = luaL_checknumber(L, 2);
	double nearp = luaL_checknumber(L, 3);
	double farp = luaL_checknumber(L, 4);
	double aspect = luaL_checknumber(L, 5);

	double top = nearp * std::tan(fov / 2);
	double bottom = -1 * top;
	double right = top * aspect;
	double left = -1 * right;

	m_set(L, 1, 1, 2 * nearp / (right - left));
	m_set(L, 1, 2, 0);
	m_set(L, 1, 3, (right + left) / (right - left));
	m_set(L, 1, 4, 0);

	m_set(L, 1, 5, 0);
	m_set(L, 1, 6, 2 * nearp / (top - bottom));
	m_set(L, 1, 7, (top + bottom) / (top - bottom));
	m_set(L, 1, 8, 0);

	m_set(L, 1, 9, 0);
	m_set(L, 1, 10, 0);
	m_set(L, 1, 11, -1 * (farp + nearp) / (farp - nearp));
	m_set(L, 1, 12, -2 * farp * nearp / (farp - nearp));

	m_set(L, 1, 13, 0);
	m_set(L, 1, 14, 0);
	m_set(L, 1, 15, -1);
	m_set(L, 1, 16, 0);

	return 0;
}

static int w_setOrthographicMatrix(lua_State *L){
	luaL_checktype(L, 1, LUA_TTABLE);

	double fov = luaL_checknumber(L, 2);
	double size = luaL_checknumber(L, 3);
	double nearp = luaL_checknumber(L, 4);
	double farp = luaL_checknumber(L, 5);
	double aspect = luaL_checknumber(L, 6);

	double top = size * std::tan(fov / 2);
	double bottom = -1 * top;
	double right = top * aspect;
	double left = -1 * right;

	m_set(L, 1, 1, 2 / (right - left));
	m_set(L, 1, 2, 0);
	m_set(L, 1, 3, 0);
	m_set(L, 1, 4, -1 * (right + left) / (right - left));

	m_set(L, 1, 5, 0);
	m_set(L, 1, 6, 2 / (top - bottom));
	m_set(L, 1, 7, 0);
	m_set(L, 1, 8, -1 * (top + bottom) / (top - bottom));

	m_set(L, 1, 9, 0);
	m_set(L, 1, 10, 0);
	m_set(L, 1, 11, -2 / (farp - nearp));
	m_set(L, 1, 12, -(farp + nearp) / (farp - nearp));

	m_set(L, 1, 13, 0);
	m_set(L, 1, 14, 0);
	m_set(L, 1, 15, 0);
	m_set(L, 1, 16, 1);

	return 0;
}

static int w_setViewMatrix(lua_State *L){
	luaL_checktype(L, 1, LUA_TTABLE);

	double eye[3], target[3], up[3];
	checkVec3(L, 2, eye);
	checkVec3(L, 3, target);
	checkVec3(L, 4, up);

	double z1 = eye[0] - target[0], z2 = eye[1] - target[1], z3 = eye[2] - target[2];
	vecNormalize(z1, z2, z3);

	double x1, x2, x3;
	vecCross(up[0], up[1], up[2], z1, z2, z3, x1, x2, x3);
	vecNormalize(x1, x2, x3);

	double y1, y2, y3;
	vecCross(z1, z2, z3, x1, x2, x3, y1, y2, y3);

	m_set(L, 1, 1, x1); m_set(L, 1, 2, x2); m_set(L, 1, 3, x3);
	m_set(L, 1, 4, -1 * vecDot(x1, x2, x3, eye[0], eye[1], eye[2]));

	m_set(L, 1, 5, y1); m_set(L, 1, 6, y2); m_set(L, 1, 7, y3);
	m_set(L, 1, 8, -1 * vecDot(y1, y2, y3, eye[0], eye[1], eye[2]));

	m_set(L, 1, 9, z1); m_set(L, 1, 10, z2); m_set(L, 1, 11, z3);
	m_set(L, 1, 12, -1 * vecDot(z1, z2, z3, eye[0], eye[1], eye[2]));

	m_set(L, 1, 13, 0);
	m_set(L, 1, 14, 0);
	m_set(L, 1, 15, 0);
	m_set(L, 1, 16, 1);

	return 0;
}

void MGOLD_setMatricesFunc(lua_State *L){
	lua_newtable(L);

	lua_pushvalue(L, -1);
	lua_setfield(L, -2, "__index");

	lua_pushcfunction(L, w_tostring);
	lua_setfield(L, -2, "__tostring");

	lua_pushvalue(L, -1);
	lua_pushcclosure(L, w_newMatrix, 1);
	lua_setfield(L, -2, "newMatrix");

	lua_pushcfunction(L, w_setTransformationMatrix);
	lua_setfield(L, -2, "setTransformationMatrix");

	lua_pushcfunction(L, w_getScale);
	lua_setfield(L, -2, "getScale");

	lua_pushcfunction(L, w_lookAtFrom);
	lua_setfield(L, -2, "lookAtFrom");

	lua_pushcfunction(L, w_setProjectionMatrix);
	lua_setfield(L, -2, "setProjectionMatrix");

	lua_pushcfunction(L, w_setOrthographicMatrix);
	lua_setfield(L, -2, "setOrthographicMatrix");

	lua_pushcfunction(L, w_setViewMatrix);
	lua_setfield(L, -2, "setViewMatrix");
}

}
}
