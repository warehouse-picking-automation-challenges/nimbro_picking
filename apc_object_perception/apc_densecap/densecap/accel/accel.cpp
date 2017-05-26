// Accelerated subroutines for DenseCap
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "nms.h"

#include <TH.h>
#include <luaT.h>

static int nms(lua_State* L)
{
	THFloatTensor* boxes = (THFloatTensor*)luaT_checkudata(L, 1, "torch.FloatTensor");

	THArgCheck(THFloatTensor_isContiguous(boxes), 1, "boxes tensor needs to be contiguous, sorry");
	THArgCheck(THFloatTensor_size(boxes, 1) == 5, 1, "boxes tensor needs 5 columns (x1, y1, x2, y2, score)");

	float max_iou = lua_tonumber(L, 2);

	int max_boxes = -1;
	if(lua_gettop(L) > 2 && !lua_isnil(L, 3))
		max_boxes = lua_tointeger(L, 3);

	std::vector<int> indices = accel::nms(
		reinterpret_cast<const accel::NMSRectangle*>(THFloatTensor_data(boxes)),
		THFloatTensor_size(boxes, 0),
		max_iou, max_boxes
	);

	THLongTensor* ret = THLongTensor_newWithSize1d(indices.size());
	auto retData = THLongTensor_data(ret);
	for(std::size_t i = 0; i < indices.size(); ++i)
	{
		retData[i] = indices[i] + 1; // LUA-style indices
	}

	luaT_pushudata(L, ret, "torch.LongTensor");

	return 1;
}

extern "C"
{

static const luaL_Reg gFuncs[] = {
	{"nms", nms},
	{NULL, NULL}
};

DLL_EXPORT int luaopen_libdensecap_accel(lua_State* L)
{
	lua_newtable(L);

	lua_pushvalue(L, -1);
	lua_setglobal(L, "densecap_accel");

	luaT_setfuncs(L, gFuncs, 0);

	return 0;
}

}
