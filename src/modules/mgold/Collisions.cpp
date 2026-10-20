#include "modules/mgold/Collisions.h"

#include <cmath>
#include <cstdint>
#include <vector>
#include <unordered_map>
#include <algorithm>
#include <limits>

#if LUA_VERSION_NUM >= 502
#define m_objlen(L, i) lua_rawlen(L, i)
#else
#define m_objlen(L, i) lua_objlen(L, i)
#endif

namespace love
{

namespace mgold
{

// ---------------------------------------------------------------------------------------------
// matematica de vetores (substitui mgi3.vectors, tudo inline e sem alocacao)
// ---------------------------------------------------------------------------------------------

struct V3{
	double x, y, z;
};

static inline V3 operator+(const V3 &a, const V3 &b){ return V3{a.x + b.x, a.y + b.y, a.z + b.z}; }
static inline V3 operator-(const V3 &a, const V3 &b){ return V3{a.x - b.x, a.y - b.y, a.z - b.z}; }
static inline V3 operator*(const V3 &a, double s){ return V3{a.x * s, a.y * s, a.z * s}; }

static inline double dot(const V3 &a, const V3 &b){
	return a.x * b.x + a.y * b.y + a.z * b.z;
}

static inline V3 cross(const V3 &a, const V3 &b){
	return V3{a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

static inline double length(const V3 &a){
	return std::sqrt(a.x * a.x + a.y * a.y + a.z * a.z);
}

static inline V3 normalize(const V3 &a){
	double len = length(a);
	if (len == 0.0)
		return V3{0.0, 0.0, 0.0};
	return V3{a.x / len, a.y / len, a.z / len};
}

static inline V3 loadV3(const double *p){
	return V3{p[0], p[1], p[2]};
}

struct Hit{
	double len;
	V3 p;
	V3 n;
};

// ---------------------------------------------------------------------------------------------
// cache no mundo (um por modelo, guardado como userdata dentro da tabela do modelo)
// ---------------------------------------------------------------------------------------------
//
//   tris[t*18 + 0..8]   = x0,y0,z0, x1,y1,z1, x2,y2,z2   (posicoes em world space)
//   tris[t*18 + 9..11]  = nx,ny,nz                       (normal da face)
//   tris[t*18 + 12..14] = minX,minY,minZ                 (AABB do triangulo)
//   tris[t*18 + 15..17] = maxX,maxY,maxZ

static const int STRIDE = 18;
static const double EPS = 1e-5;
static const double TINY = 2.2204460492503131e-16;

static const char *CACHE_META = "mgold.CollisionCache";
static const char *CACHE_FIELD = "collisionCache";

// celulas inteiras empacotadas em um uint64 (sem aliasing, sem strings)
static inline uint64_t zoneKey(long long cx, long long cy, long long cz){
	const long long OFF = 1048576; // 2^20
	const uint64_t SPAN = 2097152; // 2^21
	return (((uint64_t) (cx + OFF)) * SPAN + (uint64_t) (cy + OFF)) * SPAN + (uint64_t) (cz + OFF);
}

struct CollisionCache{
	std::vector<double> tris;
	std::vector<int> allTris;
	int triCount = 0;
	double aabbMin[3] = {0, 0, 0};
	double aabbMax[3] = {0, 0, 0};

	bool hasZones = false;
	double zoneSize = 0;
	double zoneInv = 0;
	std::unordered_map<uint64_t, std::vector<int> > zones;
	std::vector<uint32_t> seen;
	uint32_t stamp = 0;
	std::vector<int> candidates; // scratch reaproveitado entre queries
};

static int w_cache_gc(lua_State *L){
	CollisionCache **p = (CollisionCache **) lua_touserdata(L, 1);
	if (p && *p){
		delete *p;
		*p = nullptr;
	}
	return 0;
}

static int abs_index(lua_State *L, int index){
	if (index > 0 || index <= LUA_REGISTRYINDEX)
		return index;

	return lua_gettop(L) + index + 1;
}

// le t[i] respeitando __index (vec3 do lvector pode nao ser uma tabela pura)
static double getNum(lua_State *L, int idx, int i){
	idx = abs_index(L, idx);
	lua_pushinteger(L, i);
	lua_gettable(L, idx);
	double v = lua_tonumber(L, -1);
	lua_pop(L, 1);
	return v;
}

static V3 getVec3(lua_State *L, int idx){
	idx = abs_index(L, idx);
	return V3{getNum(L, idx, 1), getNum(L, idx, 2), getNum(L, idx, 3)};
}

// self[field] como vec3; se nao existir devolve def
static V3 getVec3Field(lua_State *L, int idx, const char *field, const V3 &def){
	idx = abs_index(L, idx);
	lua_getfield(L, idx, field);
	V3 r = def;
	if (!lua_isnoneornil(L, -1))
		r = getVec3(L, -1);
	lua_pop(L, 1);
	return r;
}

static CollisionCache *getCache(lua_State *L, int idx){
	idx = abs_index(L, idx);
	CollisionCache *cache = nullptr;

	lua_getfield(L, idx, CACHE_FIELD);
	if (lua_isuserdata(L, -1) && lua_getmetatable(L, -1)){
		luaL_getmetatable(L, CACHE_META);
		bool same = lua_rawequal(L, -1, -2) != 0;
		lua_pop(L, 2);
		if (same)
			cache = *(CollisionCache **) lua_touserdata(L, -1);
	}
	lua_pop(L, 1);

	return cache;
}

static CollisionCache *createCache(lua_State *L, int idx){
	idx = abs_index(L, idx);

	CollisionCache **p = (CollisionCache **) lua_newuserdata(L, sizeof(CollisionCache *));
	*p = new CollisionCache();
	luaL_getmetatable(L, CACHE_META);
	lua_setmetatable(L, -2);
	lua_setfield(L, idx, CACHE_FIELD);

	return *p;
}

static inline void rotateXYZWithTrig(double &x, double &y, double &z, double cx, double sx, double cy, double sy, double cz, double sz){
	double nx, ny, nz;

	// rotate X
	ny = y * cx - z * sx;
	nz = y * sx + z * cx;
	y = ny; z = nz;

	// rotate Y
	nx = x * cy + z * sy;
	nz = -x * sy + z * cy;
	x = nx; z = nz;

	// rotate Z
	nx = x * cz - y * sz;
	ny = x * sz + y * cz;
	x = nx; y = ny;
}

// recalcula triangulos em world space + AABB do modelo
static void updateWorldVerts(lua_State *L, int idx, CollisionCache *cache){
	idx = abs_index(L, idx);

	lua_getfield(L, idx, "mesh");
	if (!lua_istable(L, -1))
		luaL_error(L, "collisions: model.mesh must be a table");
	lua_getfield(L, -1, "verts");
	if (!lua_istable(L, -1))
		luaL_error(L, "collisions: model.mesh.verts must be a table");
	int vt = lua_gettop(L);

	int vertCount = (int) m_objlen(L, vt);
	int triCount = vertCount / 3;

	std::vector<double> verts((size_t) triCount * 9);
	for (int i = 0; i < triCount * 3; i++){
		lua_rawgeti(L, vt, i + 1);
		for (int k = 0; k < 3; k++){
			lua_rawgeti(L, -1, k + 1);
			verts[(size_t) i * 3 + k] = lua_tonumber(L, -1);
			lua_pop(L, 1);
		}
		lua_pop(L, 1);
	}
	lua_pop(L, 2); // verts, mesh

	V3 pos = getVec3Field(L, idx, "position", V3{0, 0, 0});
	V3 scl = getVec3Field(L, idx, "scale", V3{1, 1, 1});
	V3 rot = getVec3Field(L, idx, "rotation", V3{0, 0, 0});

	// sin/cos UMA vez por atualizacao
	double cx = std::cos(rot.x), sx = std::sin(rot.x);
	double cy = std::cos(rot.y), sy = std::sin(rot.y);
	double cz = std::cos(rot.z), sz = std::sin(rot.z);

	cache->tris.resize((size_t) triCount * STRIDE);
	double *tris = cache->tris.data();

	double mn[3] = {std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()};
	double mx[3] = {-std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity()};

	for (int t = 0; t < triCount; t++){
		V3 p[3];
		for (int k = 0; k < 3; k++){
			const double *v = &verts[((size_t) t * 3 + k) * 3];
			double x = v[0] * scl.x, y = v[1] * scl.y, z = v[2] * scl.z;
			rotateXYZWithTrig(x, y, z, cx, sx, cy, sy, cz, sz);
			p[k] = V3{x + pos.x, y + pos.y, z + pos.z};
		}

		// normal da face (NAO a normal suave do vertice)
		V3 n = normalize(cross(p[1] - p[0], p[2] - p[0]));

		double *o = tris + (size_t) t * STRIDE;
		for (int k = 0; k < 3; k++){
			o[k * 3 + 0] = p[k].x;
			o[k * 3 + 1] = p[k].y;
			o[k * 3 + 2] = p[k].z;
		}
		o[9] = n.x; o[10] = n.y; o[11] = n.z;

		o[12] = std::min(p[0].x, std::min(p[1].x, p[2].x));
		o[13] = std::min(p[0].y, std::min(p[1].y, p[2].y));
		o[14] = std::min(p[0].z, std::min(p[1].z, p[2].z));
		o[15] = std::max(p[0].x, std::max(p[1].x, p[2].x));
		o[16] = std::max(p[0].y, std::max(p[1].y, p[2].y));
		o[17] = std::max(p[0].z, std::max(p[1].z, p[2].z));

		for (int k = 0; k < 3; k++){
			if (o[12 + k] < mn[k]) mn[k] = o[12 + k];
			if (o[15 + k] > mx[k]) mx[k] = o[15 + k];
		}
	}

	cache->allTris.resize(triCount);
	for (int i = 0; i < triCount; i++)
		cache->allTris[i] = i;

	cache->triCount = triCount;
	for (int k = 0; k < 3; k++){
		cache->aabbMin[k] = mn[k];
		cache->aabbMax[k] = mx[k];
	}

	// as zonas antigas ficam invalidas
	cache->hasZones = false;
	cache->zones.clear();

	lua_pushboolean(L, 0);
	lua_setfield(L, idx, "collisionDirty");
}

// garante que o cache existe e esta valido
static CollisionCache *ensureWorld(lua_State *L, int idx){
	idx = abs_index(L, idx);
	luaL_checktype(L, idx, LUA_TTABLE);

	CollisionCache *cache = getCache(L, idx);
	bool fresh = false;
	if (!cache){
		cache = createCache(L, idx);
		fresh = true;
	}

	lua_getfield(L, idx, "collisionDirty");
	bool clean = lua_isboolean(L, -1) && !lua_toboolean(L, -1);
	lua_pop(L, 1);

	if (fresh || !clean)
		updateWorldVerts(L, idx, cache);

	return cache;
}

// ---------------------------------------------------------------------------------------------
// broad phase
// ---------------------------------------------------------------------------------------------

static bool broadPhase(const CollisionCache *c, const double *qmin, const double *qmax){
	return !(qmax[0] < c->aabbMin[0] || qmin[0] > c->aabbMax[0] ||
	         qmax[1] < c->aabbMin[1] || qmin[1] > c->aabbMax[1] ||
	         qmax[2] < c->aabbMin[2] || qmin[2] > c->aabbMax[2]);
}

// devolve a lista de triangulos candidatos que tocam a AABB
static const std::vector<int> &getCandidates(CollisionCache *c, const double *qmin, const double *qmax){
	if (!c->hasZones)
		return c->allTris;

	double inv = c->zoneInv;
	long long cx1 = (long long) std::floor(qmin[0] * inv), cx2 = (long long) std::floor(qmax[0] * inv);
	long long cy1 = (long long) std::floor(qmin[1] * inv), cy2 = (long long) std::floor(qmax[1] * inv);
	long long cz1 = (long long) std::floor(qmin[2] * inv), cz2 = (long long) std::floor(qmax[2] * inv);

	// consulta gigante: mais barato testar tudo do que varrer milhoes de celulas
	double cells = (double) (cx2 - cx1 + 1) * (double) (cy2 - cy1 + 1) * (double) (cz2 - cz1 + 1);
	if (cells > 4096.0)
		return c->allTris;

	// dedupe por carimbo, sem alocar "seen" por query
	uint32_t stamp = ++c->stamp;
	if (stamp == 0){
		std::fill(c->seen.begin(), c->seen.end(), 0);
		stamp = c->stamp = 1;
	}

	std::vector<int> &list = c->candidates;
	list.clear();

	for (long long cx = cx1; cx <= cx2; cx++)
		for (long long cy = cy1; cy <= cy2; cy++)
			for (long long cz = cz1; cz <= cz2; cz++){
				auto it = c->zones.find(zoneKey(cx, cy, cz));
				if (it == c->zones.end())
					continue;
				const std::vector<int> &zone = it->second;
				for (size_t i = 0; i < zone.size(); i++){
					int t = zone[i];
					if (c->seen[t] != stamp){
						c->seen[t] = stamp;
						list.push_back(t);
					}
				}
			}

	return list;
}

// ---------------------------------------------------------------------------------------------
// narrow phase
// ---------------------------------------------------------------------------------------------

static V3 closestPointOnLineSegment(const V3 &a, const V3 &b, const V3 &p){
	V3 ab = b - a;
	double denom = dot(ab, ab);
	double t = denom > 0.0 ? dot(p - a, ab) / denom : 0.0;
	t = std::min(1.0, std::max(0.0, t));
	return a + ab * t;
}

// ray - triangle (CPML / lighthouse3d)
static bool triangleRay(const V3 &p0, const V3 &p1, const V3 &p2, const V3 &n, const V3 &src, const V3 &dir, Hit &out){
	V3 e1 = p1 - p0;
	V3 e2 = p2 - p0;
	V3 h = cross(dir, e2);
	double a = dot(h, e1);

	if (std::fabs(a) <= TINY)
		return false;

	V3 s = src - p0;
	double u = dot(s, h) / a;
	if (u < 0 || u > 1)
		return false;

	V3 q = cross(s, e1);
	double v = dot(dir, q) / a;
	if (v < 0 || u + v > 1)
		return false;

	double t = dot(q, e2) / a;
	if (t >= TINY){
		out.len = t;
		out.p = src + dir * t;
		out.n = n;
		return true;
	}
	return false;
}

static inline bool finishHit(const V3 &src, const V3 &itx, const V3 &triN, Hit &out){
	V3 d = src - itx;
	out.p = itx;
	if (d.x == 0 && d.y == 0 && d.z == 0){
		// esfera exatamente em cima do triangulo: usa a normal da face
		out.len = 0;
		out.n = triN;
	}
	else{
		out.len = length(d);
		out.n = d;
	}
	return true;
}

static inline bool insideTriangle(const V3 &p0, const V3 &p1, const V3 &p2, const V3 &n, const V3 &itx){
	V3 c0 = cross(itx - p0, p1 - p0);
	V3 c1 = cross(itx - p1, p2 - p1);
	V3 c2 = cross(itx - p2, p0 - p2);
	return dot(c0, n) <= 0 && dot(c1, n) <= 0 && dot(c2, n) <= 0;
}

// esfera - triangulo (wickedengine)
static bool triangleSphere(const V3 &p0, const V3 &p1, const V3 &p2, const V3 &tn, const V3 &src, double radius, Hit &out){
	double dist = dot(src - p0, tn);
	if (dist < -radius || dist > radius)
		return false;

	V3 itx = src - tn * dist;

	if (insideTriangle(p0, p1, p2, tn, itx))
		return finishHit(src, itx, tn, out);

	// itx fora do triangulo: ponto mais proximo em cada aresta
	double radiussq = radius * radius;
	bool found = false;
	double smallest = 0;

	const V3 *edges[3][2] = {{&p0, &p1}, {&p1, &p2}, {&p2, &p0}};
	for (int i = 0; i < 3; i++){
		V3 c = closestPointOnLineSegment(*edges[i][0], *edges[i][1], src);
		V3 d = src - c;
		double dsq = dot(d, d);
		if (dsq <= radiussq && (!found || dsq < smallest)){
			found = true;
			smallest = dsq;
			itx = c;
		}
	}

	if (found)
		return finishHit(src, itx, tn, out);

	return false;
}

// ponto mais proximo do triangulo
static bool trianglePoint(const V3 &p0, const V3 &p1, const V3 &p2, const V3 &tn, const V3 &src, Hit &out){
	double dist = dot(src - p0, tn);
	V3 itx = src - tn * dist;

	if (insideTriangle(p0, p1, p2, tn, itx))
		return finishHit(src, itx, tn, out);

	const V3 *edges[3][2] = {{&p0, &p1}, {&p1, &p2}, {&p2, &p0}};
	double smallest = 0;
	for (int i = 0; i < 3; i++){
		V3 c = closestPointOnLineSegment(*edges[i][0], *edges[i][1], src);
		V3 d = src - c;
		double dsq = dot(d, d);
		if (i == 0 || dsq < smallest){
			smallest = dsq;
			itx = c;
		}
	}

	return finishHit(src, itx, tn, out);
}

// capsula - triangulo (a, b = extremos da linha central, capn = direcao da capsula)
static bool triangleCapsule(const V3 &p0, const V3 &p1, const V3 &p2, const V3 &n, const V3 &base, const V3 &a, const V3 &b, const V3 &capn, double radius, Hit &out){
	double dotOfNormals = std::fabs(dot(n, capn));

	// capsula paralela ao triangulo: usa um ponto qualquer dele
	V3 ref = p0;

	if (dotOfNormals > 0){
		double t = dot(n, (p0 - base) * (1.0 / dotOfNormals));
		V3 planeItx = base + capn * t;

		Hit tmp;
		trianglePoint(p0, p1, p2, n, planeItx, tmp);
		ref = tmp.p;
	}

	V3 c = closestPointOnLineSegment(a, b, ref);
	return triangleSphere(p0, p1, p2, n, c, radius, out);
}

// ---------------------------------------------------------------------------------------------
// helpers de retorno
// ---------------------------------------------------------------------------------------------

static int pushHit(lua_State *L, const Hit &h){
	V3 n = normalize(h.n);
	lua_pushnumber(L, h.len);
	lua_pushnumber(L, h.p.x);
	lua_pushnumber(L, h.p.y);
	lua_pushnumber(L, h.p.z);
	lua_pushnumber(L, n.x);
	lua_pushnumber(L, n.y);
	lua_pushnumber(L, n.z);
	return 7;
}

static inline bool triOverlaps(const double *o, const double *qmin, const double *qmax){
	return o[12] <= qmax[0] && o[15] >= qmin[0] &&
	       o[13] <= qmax[1] && o[16] >= qmin[1] &&
	       o[14] <= qmax[2] && o[17] >= qmin[2];
}

// ---------------------------------------------------------------------------------------------
// cache / zonas / utilidades de modelo
// ---------------------------------------------------------------------------------------------

static int w_prepare(lua_State *L){
	ensureWorld(L, 1);
	return 0;
}

static int w_invalidate(lua_State *L){
	luaL_checktype(L, 1, LUA_TTABLE);

	lua_pushboolean(L, 1);
	lua_setfield(L, 1, "collisionDirty");

	CollisionCache *cache = getCache(L, 1);
	if (cache){
		cache->hasZones = false;
		cache->zones.clear();
	}
	return 0;
}

static int w_createCollisionZones(lua_State *L){
	double zoneSize = luaL_checknumber(L, 2);
	if (zoneSize <= 0)
		return luaL_argerror(L, 2, "zoneSize must be greater than 0");

	CollisionCache *c = ensureWorld(L, 1);

	double inv = 1.0 / zoneSize;
	c->zones.clear();

	for (int t = 0; t < c->triCount; t++){
		const double *o = &c->tris[(size_t) t * STRIDE];

		long long cx1 = (long long) std::floor(o[12] * inv), cx2 = (long long) std::floor(o[15] * inv);
		long long cy1 = (long long) std::floor(o[13] * inv), cy2 = (long long) std::floor(o[16] * inv);
		long long cz1 = (long long) std::floor(o[14] * inv), cz2 = (long long) std::floor(o[17] * inv);

		// o triangulo entra em toda celula que a AABB dele toca
		for (long long cx = cx1; cx <= cx2; cx++)
			for (long long cy = cy1; cy <= cy2; cy++)
				for (long long cz = cz1; cz <= cz2; cz++)
					c->zones[zoneKey(cx, cy, cz)].push_back(t);
	}

	c->zoneSize = zoneSize;
	c->zoneInv = inv;
	c->hasZones = true;
	c->seen.assign(c->triCount, 0);
	c->stamp = 0;

	lua_pushinteger(L, (lua_Integer) c->zones.size());
	return 1;
}

// ---------------------------------------------------------------------------------------------
// queries
// ---------------------------------------------------------------------------------------------

static int w_rayIntersection(lua_State *L){
	V3 src = V3{luaL_checknumber(L, 2), luaL_checknumber(L, 3), luaL_checknumber(L, 4)};
	V3 dir = V3{luaL_checknumber(L, 5), luaL_checknumber(L, 6), luaL_checknumber(L, 7)};

	CollisionCache *c = ensureWorld(L, 1);

	bool found = false;
	Hit best = {};

	for (int t = 0; t < c->triCount; t++){
		const double *o = &c->tris[(size_t) t * STRIDE];
		Hit h;
		if (triangleRay(loadV3(o), loadV3(o + 3), loadV3(o + 6), loadV3(o + 9), src, dir, h) && (!found || h.len < best.len)){
			best = h;
			found = true;
		}
	}

	return found ? pushHit(L, best) : 0;
}

static int w_sphereIntersection(lua_State *L){
	V3 src = V3{luaL_checknumber(L, 2), luaL_checknumber(L, 3), luaL_checknumber(L, 4)};
	double radius = luaL_checknumber(L, 5);

	CollisionCache *c = ensureWorld(L, 1);

	double r = radius + EPS;
	double qmin[3] = {src.x - r, src.y - r, src.z - r};
	double qmax[3] = {src.x + r, src.y + r, src.z + r};

	if (!broadPhase(c, qmin, qmax))
		return 0;

	const std::vector<int> &list = getCandidates(c, qmin, qmax);

	bool found = false;
	Hit best = {};

	for (size_t i = 0; i < list.size(); i++){
		const double *o = &c->tris[(size_t) list[i] * STRIDE];
		if (!triOverlaps(o, qmin, qmax))
			continue;

		Hit h;
		if (triangleSphere(loadV3(o), loadV3(o + 3), loadV3(o + 6), loadV3(o + 9), src, radius, h) && (!found || h.len < best.len)){
			best = h;
			found = true;
		}
	}

	return found ? pushHit(L, best) : 0;
}

static int w_closestPoint(lua_State *L){
	V3 src = V3{luaL_checknumber(L, 2), luaL_checknumber(L, 3), luaL_checknumber(L, 4)};

	CollisionCache *c = ensureWorld(L, 1);

	double q[3] = {src.x, src.y, src.z};
	const std::vector<int> &list = getCandidates(c, q, q);

	bool found = false;
	Hit best = {};

	for (size_t i = 0; i < list.size(); i++){
		const double *o = &c->tris[(size_t) list[i] * STRIDE];
		Hit h;
		if (trianglePoint(loadV3(o), loadV3(o + 3), loadV3(o + 6), loadV3(o + 9), src, h) && (!found || h.len < best.len)){
			best = h;
			found = true;
		}
	}

	return found ? pushHit(L, best) : 0;
}

static int w_capsuleIntersection(lua_State *L){
	V3 tip = V3{luaL_checknumber(L, 2), luaL_checknumber(L, 3), luaL_checknumber(L, 4)};
	V3 base = V3{luaL_checknumber(L, 5), luaL_checknumber(L, 6), luaL_checknumber(L, 7)};
	double radius = luaL_checknumber(L, 8);

	CollisionCache *c = ensureWorld(L, 1);

	double r = radius + EPS;
	double qmin[3] = {std::min(tip.x, base.x) - r, std::min(tip.y, base.y) - r, std::min(tip.z, base.z) - r};
	double qmax[3] = {std::max(tip.x, base.x) + r, std::max(tip.y, base.y) + r, std::max(tip.z, base.z) + r};

	// rejeicao no nivel do modelo
	if (!broadPhase(c, qmin, qmax))
		return 0;

	V3 norm = normalize(tip - base);
	V3 a = base + norm * radius;
	V3 b = tip - norm * radius;

	const std::vector<int> &list = getCandidates(c, qmin, qmax);

	bool found = false;
	Hit best = {};

	for (size_t i = 0; i < list.size(); i++){
		const double *o = &c->tris[(size_t) list[i] * STRIDE];
		if (!triOverlaps(o, qmin, qmax))
			continue;

		Hit h;
		if (triangleCapsule(loadV3(o), loadV3(o + 3), loadV3(o + 6), loadV3(o + 9), base, a, b, norm, radius, h) && (!found || h.len < best.len)){
			best = h;
			found = true;
		}
	}

	return found ? pushHit(L, best) : 0;
}

// ---------------------------------------------------------------------------------------------
// AABB
// ---------------------------------------------------------------------------------------------

static int w_generateAABB(lua_State *L){
	luaL_checktype(L, 1, LUA_TTABLE);

	lua_getfield(L, 1, "mesh");
	if (!lua_istable(L, -1))
		return luaL_error(L, "collisions: model.mesh must be a table");
	lua_getfield(L, -1, "verts");
	if (!lua_istable(L, -1))
		return luaL_error(L, "collisions: model.mesh.verts must be a table");
	int vt = lua_gettop(L);

	double mn[3] = {std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()};
	double mx[3] = {-std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity()};

	int n = (int) m_objlen(L, vt);
	for (int i = 1; i <= n; i++){
		lua_rawgeti(L, vt, i);
		for (int k = 0; k < 3; k++){
			lua_rawgeti(L, -1, k + 1);
			double v = lua_tonumber(L, -1);
			lua_pop(L, 1);
			if (v < mn[k]) mn[k] = v;
			if (v > mx[k]) mx[k] = v;
		}
		lua_pop(L, 1);
	}
	lua_pop(L, 2);

	lua_newtable(L); // aabb

	lua_newtable(L);
	for (int k = 0; k < 3; k++){
		lua_pushnumber(L, mn[k]);
		lua_rawseti(L, -2, k + 1);
	}
	lua_setfield(L, -2, "min");

	lua_newtable(L);
	for (int k = 0; k < 3; k++){
		lua_pushnumber(L, mx[k]);
		lua_rawseti(L, -2, k + 1);
	}
	lua_setfield(L, -2, "max");

	lua_pushvalue(L, -1);
	lua_setfield(L, 1, "aabb");

	return 1;
}

// AABB do modelo (posicao + escala), com min/max trocados se a escala for negativa
static void getModelAABB(lua_State *L, int idx, V3 &mn, V3 &mx){
	idx = abs_index(L, idx);
	luaL_checktype(L, idx, LUA_TTABLE);

	lua_getfield(L, idx, "aabb");
	if (!lua_istable(L, -1))
		luaL_error(L, "collisions: model has no aabb (call generateAABB first)");
	int ab = lua_gettop(L);

	lua_getfield(L, ab, "min");
	V3 amin = getVec3(L, -1);
	lua_pop(L, 1);
	lua_getfield(L, ab, "max");
	V3 amax = getVec3(L, -1);
	lua_pop(L, 2);

	V3 s = getVec3Field(L, idx, "scale", V3{1, 1, 1});
	V3 p = getVec3Field(L, idx, "position", V3{0, 0, 0});

	double x1 = amin.x * s.x + p.x, x2 = amax.x * s.x + p.x;
	double y1 = amin.y * s.y + p.y, y2 = amax.y * s.y + p.y;
	double z1 = amin.z * s.z + p.z, z2 = amax.z * s.z + p.z;

	mn = V3{std::min(x1, x2), std::min(y1, y2), std::min(z1, z2)};
	mx = V3{std::max(x1, x2), std::max(y1, y2), std::max(z1, z2)};
}

static int w_sphereIntersectsAABB(lua_State *L){
	V3 c = V3{luaL_checknumber(L, 2), luaL_checknumber(L, 3), luaL_checknumber(L, 4)};
	double radius = luaL_checknumber(L, 5);

	V3 mn, mx;
	getModelAABB(L, 1, mn, mx);

	bool hit = (c.x + radius >= mn.x && c.x - radius <= mx.x) &&
	           (c.y + radius >= mn.y && c.y - radius <= mx.y) &&
	           (c.z + radius >= mn.z && c.z - radius <= mx.z);
	lua_pushboolean(L, hit);
	return 1;
}

static int w_capsuleIntersectsAABB(lua_State *L){
	V3 p1 = V3{luaL_checknumber(L, 2), luaL_checknumber(L, 3), luaL_checknumber(L, 4)};
	V3 p2 = V3{luaL_checknumber(L, 5), luaL_checknumber(L, 6), luaL_checknumber(L, 7)};
	double radius = luaL_checknumber(L, 8);

	V3 mn, mx;
	getModelAABB(L, 1, mn, mx);

	double cminx = std::min(p1.x, p2.x) - radius, cmaxx = std::max(p1.x, p2.x) + radius;
	double cminy = std::min(p1.y, p2.y) - radius, cmaxy = std::max(p1.y, p2.y) + radius;
	double cminz = std::min(p1.z, p2.z) - radius, cmaxz = std::max(p1.z, p2.z) + radius;

	bool hit = (cminx <= mx.x && cmaxx >= mn.x) &&
	           (cminy <= mx.y && cmaxy >= mn.y) &&
	           (cminz <= mx.z && cmaxz >= mn.z);
	lua_pushboolean(L, hit);
	return 1;
}

static int w_isIntersectionAABB(lua_State *L){
	V3 amn, amx, bmn, bmx;
	getModelAABB(L, 1, amn, amx);
	getModelAABB(L, 2, bmn, bmx);

	bool hit = amn.x <= bmx.x && amx.x >= bmn.x &&
	           amn.y <= bmx.y && amx.y >= bmn.y &&
	           amn.z <= bmx.z && amx.z >= bmn.z;
	lua_pushboolean(L, hit);
	return 1;
}

static int w_isPointInsideAABB(lua_State *L){
	V3 p = V3{luaL_checknumber(L, 2), luaL_checknumber(L, 3), luaL_checknumber(L, 4)};

	V3 mn, mx;
	getModelAABB(L, 1, mn, mx);

	bool in = p.x >= mn.x && p.x <= mx.x &&
	          p.y >= mn.y && p.y <= mx.y &&
	          p.z >= mn.z && p.z <= mx.z;
	lua_pushboolean(L, in);
	return 1;
}

static int w_getDistanceFrom(lua_State *L){
	V3 p = V3{luaL_checknumber(L, 2), luaL_checknumber(L, 3), luaL_checknumber(L, 4)};
	luaL_checktype(L, 1, LUA_TTABLE);

	V3 pos = getVec3Field(L, 1, "position", V3{0, 0, 0});
	lua_pushnumber(L, length(p - pos));
	return 1;
}

// AABB - ray (CPML)
static int w_rayIntersectionAABB(lua_State *L){
	V3 src = V3{luaL_checknumber(L, 2), luaL_checknumber(L, 3), luaL_checknumber(L, 4)};
	V3 dir = normalize(V3{luaL_checknumber(L, 5), luaL_checknumber(L, 6), luaL_checknumber(L, 7)});

	luaL_checktype(L, 1, LUA_TTABLE);

	// aqui o original NAO troca min/max por escala negativa, mantido igual
	lua_getfield(L, 1, "aabb");
	if (!lua_istable(L, -1))
		return luaL_error(L, "collisions: model has no aabb (call generateAABB first)");
	int ab = lua_gettop(L);
	lua_getfield(L, ab, "min");
	V3 amin = getVec3(L, -1);
	lua_pop(L, 1);
	lua_getfield(L, ab, "max");
	V3 amax = getVec3(L, -1);
	lua_pop(L, 2);

	V3 s = getVec3Field(L, 1, "scale", V3{1, 1, 1});
	V3 p = getVec3Field(L, 1, "position", V3{0, 0, 0});

	double t1 = (amin.x * s.x + p.x - src.x) / dir.x;
	double t2 = (amax.x * s.x + p.x - src.x) / dir.x;
	double t3 = (amin.y * s.y + p.y - src.y) / dir.y;
	double t4 = (amax.y * s.y + p.y - src.y) / dir.y;
	double t5 = (amin.z * s.z + p.z - src.z) / dir.z;
	double t6 = (amax.z * s.z + p.z - src.z) / dir.z;

	double tmin = std::max(std::max(std::min(t1, t2), std::min(t3, t4)), std::min(t5, t6));
	double tmax = std::min(std::min(std::max(t1, t2), std::max(t3, t4)), std::max(t5, t6));

	// AABB inteira atras da origem, ou raio nao toca
	if (tmax < 0 || tmin > tmax){
		lua_pushboolean(L, 0);
		return 1;
	}

	lua_pushnumber(L, tmin);
	lua_pushnumber(L, src.x + dir.x * tmin);
	lua_pushnumber(L, src.y + dir.y * tmin);
	lua_pushnumber(L, src.z + dir.z * tmin);
	return 4;
}

// ---------------------------------------------------------------------------------------------
// registro
// ---------------------------------------------------------------------------------------------

void MGOLD_setCollisionsFunc(lua_State *L){
	lua_newtable(L); // collisions

	// metatable do userdata de cache (libera o CollisionCache no gc)
	luaL_newmetatable(L, CACHE_META);
	lua_pushcfunction(L, w_cache_gc);
	lua_setfield(L, -2, "__gc");
	lua_pop(L, 1);

	struct Entry{ const char *name; lua_CFunction fn; };
	static const Entry entries[] = {
		{"prepare", w_prepare},
		{"invalidate", w_invalidate},
		{"createCollisionZones", w_createCollisionZones},
		{"rayIntersection", w_rayIntersection},
		{"sphereIntersection", w_sphereIntersection},
		{"closestPoint", w_closestPoint},
		{"capsuleIntersection", w_capsuleIntersection},
		{"generateAABB", w_generateAABB},
		{"sphereIntersectsAABB", w_sphereIntersectsAABB},
		{"capsuleIntersectsAABB", w_capsuleIntersectsAABB},
		{"isIntersectionAABB", w_isIntersectionAABB},
		{"isPointInsideAABB", w_isPointInsideAABB},
		{"getDistanceFrom", w_getDistanceFrom},
		{"rayIntersectionAABB", w_rayIntersectionAABB},
	};

	for (const Entry &e : entries){
		lua_pushcfunction(L, e.fn);
		lua_setfield(L, -2, e.name);
	}
}

}
}