// Object-class segmentation for APC
// Author: Anton Milan <anton.milan@ais.uni-bonn.de>
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include <apc_segmentation/apc_segmentation.h>

#include <ros/console.h>
#include <ros/package.h>

#ifndef APC_SEGMENTATION_MOCK

extern "C"
{
#  include <lualib.h>
#  include <lauxlib.h>
#  include <luaT.h>
}

#  include <TH/THTensor.h>

#  include <pcl/common/transforms.h>

#endif

namespace apc_segmentation
{


class APCSegmentationPrivate
{
public:
#ifndef APC_SEGMENTATION_MOCK
	std::vector<std::string> getLuaObjects()
	{
		THSetErrorHandler(&handleTorchError, this);
		THSetArgErrorHandler(&handleTorchArgError, this);

		std::vector<std::string> luaObjects;
		{
			lua_getglobal(L, "getClassInfo");

			if(lua_pcall(L, 0, 2, 0) != 0)
			{
				ROS_ERROR("LUA error:\n%s", lua_tostring(L, -1));
				throw std::runtime_error("Could not get class info");
			}

			int outIdx = lua_gettop(L);

			for(int i = 1; i < 100; ++i)
			{
				lua_pushinteger(L, i);
				lua_gettable(L, outIdx);

				if(lua_isnil(L, -1))
					break;

				if(!lua_isstring(L, -1))
				{
					ROS_ERROR("Expected string");
					throw std::runtime_error("expected string from LUA");
				}

				luaObjects.emplace_back(lua_tostring(L, -1));
				lua_pop(L, 1);
			}

			lua_settop(L, outIdx-1);
		}

		return luaObjects;
	}

	std::string lua_traceback()
	{
		luaL_traceback(L, L, NULL, 0);
		return lua_tostring(L, -1);
	}

	static void handleTorchError(const char* msg, void* data)
	{
		APCSegmentationPrivate* d = reinterpret_cast<APCSegmentationPrivate*>(data);

		ROS_ERROR("Torch error: '%s'\n", msg);
		ROS_ERROR("LUA traceback:\n%s", d->lua_traceback().c_str());
		std::abort();
	}

	static void handleTorchArgError(int idx, const char* msg, void* data)
	{
		APCSegmentationPrivate* d = reinterpret_cast<APCSegmentationPrivate*>(data);

		ROS_ERROR("Torch argument %d error: '%s'\n", idx, msg);
		ROS_ERROR("LUA traceback:\n%s", d->lua_traceback().c_str());
		std::abort();
	}

	lua_State* L = 0;
	APCSegmentation::Mode mode = APCSegmentation::MODE_SHELF;
#endif

	cv::Mat_<uint8_t> binMask;
};

APCSegmentation::APCSegmentation()
 : m_d(new APCSegmentationPrivate)
{
}

APCSegmentation::~APCSegmentation()
{
#ifndef APC_SEGMENTATION_MOCK
	THSetErrorHandler(0, 0);
	THSetArgErrorHandler(0, 0);

	if(m_d->L)
		lua_close(m_d->L);
#endif
}

void APCSegmentation::initialize(Mode mode, unsigned int gpu)
{
#ifndef APC_SEGMENTATION_MOCK
	m_d->mode = mode;

	// Create LUA interpreter
	m_d->L = luaL_newstate();
	if(!m_d->L)
		throw std::runtime_error("Could not create LUA instance");

	// Load LUA standard libraries
	luaL_openlibs(m_d->L);

	THSetErrorHandler(&APCSegmentationPrivate::handleTorchError, m_d.get());
	THSetArgErrorHandler(&APCSegmentationPrivate::handleTorchArgError, m_d.get());

	// Modify LUA path to include segmentation code
	{
		lua_getglobal(m_d->L, "package");
		lua_getfield(m_d->L, -1, "path");

		std::string path = lua_tostring(m_d->L, -1);
		path += ";" + ros::package::getPath("apc_segmentation") + "/lua/?.lua";

		lua_pop(m_d->L, 1);

		lua_pushstring(m_d->L, path.c_str());
		lua_setfield(m_d->L, -2, "path");

		lua_pop(m_d->L, 1);
	}

	ROS_DEBUG("Running apc.lua...");
	std::string scriptFile = ros::package::getPath("apc_segmentation") + "/lua/apc.lua";

	if(luaL_dofile(m_d->L, scriptFile.c_str()) != 0)
	{
		ROS_ERROR("Could not execute '%s'", scriptFile.c_str());
		ROS_ERROR("LUA error:\n%s", lua_tostring(m_d->L, -1));
		throw std::runtime_error("Could not execute LUA script");
	}

	ROS_DEBUG("Setting parameters...");
	{
		lua_getglobal(m_d->L, "setParams");

		lua_pushnumber(m_d->L, gpu + 1); //  cuda device (lua is 1-based)
		
		switch(m_d->mode)
		{
			case MODE_SHELF:
				lua_pushnumber(m_d->L, 2); //  crop mode
				lua_pushstring(m_d->L, "apc_hha_lowres_0702_train"); //  which model to use
				lua_pushnumber(m_d->L, 44); //  number of classes
				lua_pushnumber(m_d->L, 0); //  particular object class?
				lua_pushstring(m_d->L, "shelf"); //  'shelf' or 'tote'?
				break;
			case MODE_TOTE:
				lua_pushnumber(m_d->L, 2); //  crop mode
				lua_pushstring(m_d->L, "apc_hha_lowres_0629_all"); //  which model to use
				lua_pushnumber(m_d->L, 41); //  number of classes
				lua_pushnumber(m_d->L, 0); //  particular object class?
				lua_pushstring(m_d->L, "tote"); //  'shelf' or 'tote'?
				break;
			case MODE_SHELF_BACKGROUND:
				lua_pushnumber(m_d->L, 2); //  crop mode
				lua_pushstring(m_d->L, "apc_depths_lowres_0702_train"); //  which model to use
				lua_pushnumber(m_d->L, 44); //  number of classes
				lua_pushnumber(m_d->L, 0); //  particular object class?
				lua_pushstring(m_d->L, "shelf"); //  'shelf' or 'tote'?
				break;
		}

		lua_pushstring(m_d->L, ros::package::getPath("apc_segmentation").c_str());

		if(lua_pcall(m_d->L, 7, 0, 0) != 0)
		{
			ROS_ERROR("LUA error:\n%s", lua_tostring(m_d->L, -1));
			throw std::runtime_error("Could not set parameters");
		}
	}

	ROS_DEBUG("Load models ...");
	{
		lua_getglobal(m_d->L, "initAPCSegmentation");

		std::string model = ros::package::getPath("apc_segmentation");

		if(lua_pcall(m_d->L, 0, 0, 0) != 0)
		{
			ROS_ERROR("LUA error:\n%s", lua_tostring(m_d->L, -1));
			throw std::runtime_error("Could not load model");
		}
	}
#endif
}

void APCSegmentation::setBinMask(const cv::Mat_<uint8_t>& binMask)
{
	m_d->binMask = binMask;
}

APCSegmentation::Result APCSegmentation::compute(const PointCloud::ConstPtr& cloud)
{
#ifndef APC_SEGMENTATION_MOCK
	if((int)cloud->width != m_d->binMask.cols || (int)cloud->height != m_d->binMask.rows)
	{
		throw std::runtime_error("bin mask does not match cloud dimensions - did you forget to call setBinMask()?");
	}

	THSetErrorHandler(&APCSegmentationPrivate::handleTorchError, m_d.get());
	THSetArgErrorHandler(&APCSegmentationPrivate::handleTorchArgError, m_d.get());

	// Extract RGB float image and depth
	THFloatTensor* RGBTensor = THFloatTensor_newWithSize3d(
		3, cloud->height, cloud->width
	);
	THFloatTensor* DepthTensor = THFloatTensor_newWithSize3d(
		1, cloud->height, cloud->width
	);
	THByteTensor* BinMaskTensor = THByteTensor_newWithSize3d(
		1, cloud->height, cloud->width
	);

	float* rgbData = THFloatTensor_data(RGBTensor);
	float* zData = THFloatTensor_data(DepthTensor);

	{
		int CHANNEL_STRIDE = cloud->width * cloud->height;

		for(unsigned int y = 0; y < cloud->height; ++y)
		{
			for(unsigned int x = 0; x < cloud->width; ++x)
			{
				auto& p = (*cloud)(x,y);

				rgbData[0 * CHANNEL_STRIDE + y * cloud->width + x] = (1.0 / 255.0) * p.r;
				rgbData[1 * CHANNEL_STRIDE + y * cloud->width + x] = (1.0 / 255.0) * p.g;
				rgbData[2 * CHANNEL_STRIDE + y * cloud->width + x] = (1.0 / 255.0) * p.b;
				zData[0 * CHANNEL_STRIDE + y * cloud->width + x] = (1.0 / 1.0) * p.z;

			}
		}
	}

	uint8_t* maskData = THByteTensor_data(BinMaskTensor);
	for(int y = 0; y < m_d->binMask.rows; ++y)
	{
		memcpy(maskData + y * cloud->width, m_d->binMask[y], sizeof(uint8_t) * cloud->width);
	}

	// Call LUA code
	Result result;
	{
		lua_getglobal(m_d->L, "run_image");

		luaT_pushudata(m_d->L, RGBTensor, "torch.FloatTensor");
		luaT_pushudata(m_d->L, DepthTensor, "torch.FloatTensor");
		luaT_pushudata(m_d->L, BinMaskTensor, "torch.ByteTensor");

		if(lua_pcall(m_d->L, 3, 1, 0) != 0)
		{
			ROS_ERROR("LUA error:\n%s", lua_tostring(m_d->L, -1));
			throw std::runtime_error("LUA error");
		}

		int outIdx = lua_gettop(m_d->L);

		THByteTensor* thmask = (THByteTensor*)luaT_getfieldcheckudata(m_d->L, outIdx, "mask", "torch.ByteTensor");
		THFloatTensor* thpred = (THFloatTensor*)luaT_getfieldcheckudata(m_d->L, outIdx, "prediction", "torch.FloatTensor");

		ROS_INFO("Got %ldx%ld segmentation mask",
			THByteTensor_size(thmask, 0), THByteTensor_size(thmask, 1)
		);

		ROS_INFO("Got %ldx%ldx%ld prediction matrix",
			THFloatTensor_size(thpred, 0),
			THFloatTensor_size(thpred, 1),
			THFloatTensor_size(thpred, 2)
		);

		// Make explicit copy of the LUA data since it is going to be deleted
		cv::Mat_<uint8_t> mask(
			THByteTensor_size(thmask, 0), THByteTensor_size(thmask, 1),
			THByteTensor_data(thmask), THByteTensor_stride(thmask, 0)
		);
		mask.copyTo(result.mask);

		const int sizes[] = {
			(int)THFloatTensor_size(thpred, 0), (int)THFloatTensor_size(thpred, 1),
			(int)THFloatTensor_size(thpred, 2)
		};
		const size_t steps[] = {
			(size_t)THFloatTensor_stride(thpred, 0) * sizeof(float),
			(size_t)THFloatTensor_stride(thpred, 1) * sizeof(float),
			(size_t)THFloatTensor_stride(thpred, 2) * sizeof(float)
		};
		cv::Mat confidence(3, sizes, CV_32FC1, THFloatTensor_data(thpred), steps);
		confidence.copyTo(result.prediction);

		lua_settop(m_d->L, outIdx-1);

		lua_gc(m_d->L, LUA_GCCOLLECT, 0);
	}

	return result;
#else
	Result result;
	return result;
#endif
}


APCSegmentation::Result APCSegmentation::compute(const PointCloud::ConstPtr& cloud, const cv::Mat_<cv::Vec3b>& hha)
{
#ifndef APC_SEGMENTATION_MOCK
	if((int)cloud->width != m_d->binMask.cols || (int)cloud->height != m_d->binMask.rows)
	{
		throw std::runtime_error("bin mask does not match cloud dimensions - did you forget to call setBinMask()?");
	}

	THSetErrorHandler(&APCSegmentationPrivate::handleTorchError, m_d.get());
	THSetArgErrorHandler(&APCSegmentationPrivate::handleTorchArgError, m_d.get());

	// Extract RGB float image and depth
	THFloatTensor* RGBTensor = THFloatTensor_newWithSize3d(
		3, cloud->height, cloud->width
	);
// 	THFloatTensor* DepthTensor = THFloatTensor_newWithSize3d(
// 		1, cloud->height, cloud->width
// 	);
	THByteTensor* BinMaskTensor = THByteTensor_newWithSize3d(
		1, cloud->height, cloud->width
	);
	THFloatTensor* hhaTensor = THFloatTensor_newWithSize3d(
		3, cloud->height, cloud->width
	);

	float* rgbData = THFloatTensor_data(RGBTensor);
// 	float* zData = THFloatTensor_data(DepthTensor);

	{
		int CHANNEL_STRIDE = cloud->width * cloud->height;

		for(unsigned int y = 0; y < cloud->height; ++y)
		{
			for(unsigned int x = 0; x < cloud->width; ++x)
			{
				auto& p = (*cloud)(x,y);

				rgbData[0 * CHANNEL_STRIDE + y * cloud->width + x] = (1.0 / 255.0) * p.r;
				rgbData[1 * CHANNEL_STRIDE + y * cloud->width + x] = (1.0 / 255.0) * p.g;
				rgbData[2 * CHANNEL_STRIDE + y * cloud->width + x] = (1.0 / 255.0) * p.b;
// 				zData[0 * CHANNEL_STRIDE + y * cloud->width + x] = (1.0 / 1.0) * p.z;

				auto& h = hha(y,x);
				THFloatTensor_set3d(hhaTensor, 0, y, x, (1.0 / 255.0) * h[2]);
				THFloatTensor_set3d(hhaTensor, 1, y, x, (1.0 / 255.0) * h[1]);
				THFloatTensor_set3d(hhaTensor, 2, y, x, (1.0 / 255.0) * h[0]);
			}
		}
	}

	uint8_t* maskData = THByteTensor_data(BinMaskTensor);
	for(int y = 0; y < m_d->binMask.rows; ++y)
	{
		memcpy(maskData + y * cloud->width, m_d->binMask[y], sizeof(uint8_t) * cloud->width);
	}

	// Call LUA code
	Result result;
	{
		lua_getglobal(m_d->L, "run_image");

		luaT_pushudata(m_d->L, RGBTensor, "torch.FloatTensor");
		luaT_pushudata(m_d->L, hhaTensor, "torch.FloatTensor");
		luaT_pushudata(m_d->L, BinMaskTensor, "torch.ByteTensor");

		if(lua_pcall(m_d->L, 3, 1, 0) != 0)
		{
			ROS_ERROR("LUA error:\n%s", lua_tostring(m_d->L, -1));
			throw std::runtime_error("LUA error");
		}

		int outIdx = lua_gettop(m_d->L);

		THByteTensor* thmask = (THByteTensor*)luaT_getfieldcheckudata(m_d->L, outIdx, "mask", "torch.ByteTensor");
		THFloatTensor* thpred = (THFloatTensor*)luaT_getfieldcheckudata(m_d->L, outIdx, "prediction", "torch.FloatTensor");

		ROS_INFO("Got %ldx%ld segmentation mask",
			THByteTensor_size(thmask, 0), THByteTensor_size(thmask, 1)
		);

		ROS_INFO("Got %ldx%ldx%ld prediction matrix",
			THFloatTensor_size(thpred, 0),
			THFloatTensor_size(thpred, 1),
			THFloatTensor_size(thpred, 2)
		);

		// Make explicit copy of the LUA data since it is going to be deleted
		cv::Mat_<uint8_t> mask(
			THByteTensor_size(thmask, 0), THByteTensor_size(thmask, 1),
			THByteTensor_data(thmask), THByteTensor_stride(thmask, 0)
		);
		mask.copyTo(result.mask);

		const int sizes[] = {
			(int)THFloatTensor_size(thpred, 0), (int)THFloatTensor_size(thpred, 1),
			(int)THFloatTensor_size(thpred, 2)
		};
		const size_t steps[] = {
			(size_t)THFloatTensor_stride(thpred, 0) * sizeof(float),
			(size_t)THFloatTensor_stride(thpred, 1) * sizeof(float),
			(size_t)THFloatTensor_stride(thpred, 2) * sizeof(float)
		};
		cv::Mat confidence(3, sizes, CV_32FC1, THFloatTensor_data(thpred), steps);
		confidence.copyTo(result.prediction);

		lua_settop(m_d->L, outIdx-1);

		lua_gc(m_d->L, LUA_GCCOLLECT, 0);
	}

	return result;
#else
	Result result;
	return result;
#endif
}

APCSegmentation::Result APCSegmentation::segmentShelf(const PointCloud::ConstPtr& cloud)
{
#ifndef APC_SEGMENTATION_MOCK
	auto result = compute(cloud);

	result.luaObjects = m_d->getLuaObjects();

	const std::vector<std::string> backgroundClasses = {
		"box", "front_bar", "side_bar",
	};
	std::vector<int> objectIndices;

	// Find shelf index
	for(auto& obj : backgroundClasses)
	{
		auto it = std::find(result.luaObjects.begin(), result.luaObjects.end(), obj);
		if(it == result.luaObjects.end())
		{
			ROS_ERROR("Could not find object '%s' on LUA side", obj.c_str());
			throw std::runtime_error("Could not find object");
		}

		objectIndices.push_back(it - result.luaObjects.begin());
	}

	// Convert into binary mask
	for(int y = 0; y < result.mask.rows; ++y)
	{
		for(int x = 0; x < result.mask.cols; ++x)
		{
			float probShelf = 0.0f;

			for(auto& idx : objectIndices)
			{
				probShelf += result.prediction(idx, y, x);
			}

			result.mask(y,x) = (probShelf >= 0.5);
		}
	}

	return result;
#else
	Result result;
	return result;
#endif
}

APCSegmentation::Result APCSegmentation::segmentObjects(
	const PointCloud::ConstPtr& cloud, const cv::Mat_<cv::Vec3b>& hha,
	const std::vector<std::string>& objects)
{
#ifndef APC_SEGMENTATION_MOCK
	auto result = compute(cloud, hha);

	result.luaObjects = m_d->getLuaObjects();

	ROS_DEBUG("Got %d lua objects:", (int)result.luaObjects.size());
	for(size_t i = 0; i < result.luaObjects.size(); ++i)
		ROS_DEBUG(" - %20s: %lu", result.luaObjects[i].c_str(), i);

	std::vector<size_t> objectIndices;
	std::vector<int> outputIndices;

	std::vector<std::string> backgroundClasses;

	if(m_d->mode == MODE_SHELF)
	{
		backgroundClasses = {
			"box", "front_bar", "side_bar", "ground_metal", "unknown"
		};
	}
	else
	{
		backgroundClasses = {"box", "unknown"};
	}

	// Find shelf index
	for(auto& obj : backgroundClasses)
	{
		auto it = std::find(result.luaObjects.begin(), result.luaObjects.end(), obj);
		if(it == result.luaObjects.end())
		{
			ROS_ERROR("Could not find object '%s' on LUA side", obj.c_str());
			throw std::runtime_error("Could not find object");
		}

		objectIndices.push_back(it - result.luaObjects.begin());
		outputIndices.push_back(0); // background
	}

	// Find indices of our objects within the lua objects
	int outIdx = 1;
	for(auto& obj : objects)
	{
		auto it = std::find(result.luaObjects.begin(), result.luaObjects.end(), obj);
		if(it == result.luaObjects.end())
		{
			ROS_ERROR("Could not find object '%s' on LUA side", obj.c_str());
			throw std::runtime_error("Could not find object");
		}

		objectIndices.push_back(it - result.luaObjects.begin());
		outputIndices.push_back(outIdx++);
	}

	ROS_DEBUG("Object indices:");
	for(auto idx : objectIndices)
		ROS_DEBUG(" - %d", (int)idx);

	// Convert into binary mask
	result.confidence.create(result.mask.rows, result.mask.cols);

	const int predSizes[] = {(int)objects.size()+1, result.mask.rows, result.mask.cols};
	result.objectPrediction.create(3, predSizes);
	result.objectPrediction = 0.0f;

	for(int y = 0; y < result.mask.rows; ++y)
	{
		for(int x = 0; x < result.mask.cols; ++x)
		{
			float maxconf = 0;
			float sumconf = 0;
			int maxIdx = -1;

			for(size_t i = 0; i < objectIndices.size(); ++i)
			{
				float conf = result.prediction(objectIndices[i],y,x);

				if(conf > maxconf)
				{
					maxIdx = i;
					maxconf = conf;
				}

				sumconf += conf;
			}

			if(maxIdx == -1)
				maxIdx = 0;

			result.mask(y,x) = outputIndices[maxIdx];

			if(sumconf != 0)
			{
				for(size_t i = 0; i < objectIndices.size(); ++i)
					result.objectPrediction(outputIndices[i], y, x) += result.prediction(objectIndices[i], y, x) / sumconf;

				result.confidence(y,x) = maxconf / sumconf;
			}
			else
				result.confidence(y,x) = 0.0;
		}
	}

	return result;
#else
	Result result;
	return result;
#endif
}

}
