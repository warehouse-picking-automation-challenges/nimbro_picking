// APC objects
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include <apc_objects/apc_objects.h>

#include <ros/package.h>

#include <map>

#include <yaml-cpp/yaml.h>

#include <boost/filesystem.hpp>

namespace fs = boost::filesystem;

namespace apc_objects
{

static const std::vector<std::string> object_names{
	"barkely_hide_bones",
	"box",
	"cherokee_easy_tee_shirt",
	"clorox_utility_brush",
	"cloud_b_plush_bear",
	"command_hooks",
	"cool_shot_glue_sticks",
	"crayola_24_ct",
	"creativity_chenille_stems",
	"dasani_water_bottle",
	"dove_beauty_bar",
	"dr_browns_bottle_brush",
	"easter_turtle_sippy_cup",
	"elmers_washable_no_run_school_glue",
	"expo_dry_erase_board_eraser",
	"fiskars_scissors_red",
	"fitness_gear_3lb_dumbbell",
	"folgers_classic_roast_coffee",
	"front_bar",
	"ground_metal",
	"hanes_tube_socks",
	"i_am_a_bunny_book",
	"jane_eyre_dvd",
	"kleenex_paper_towels",
	"kleenex_tissue_box",
	"kyjen_squeakin_eggs_plush_puppies",
	"laugh_out_loud_joke_book",
	"oral_b_toothbrush_green",
	"oral_b_toothbrush_red",
	"peva_shower_curtain_liner",
	"platinum_pets_dog_bowl",
	"rawlings_baseball",
	"rolodex_jumbo_pencil_cup",
	"safety_first_outlet_plugs",
	"side_bar",
	"scotch_bubble_mailer",
	"scotch_duct_tape",
	"soft_white_lightbulb",
	"staples_index_cards",
	"ticonderoga_12_pencils",
	"up_glucose_bottle",
	"womens_knit_gloves",
	"woods_extension_cord",
};

static std::map<std::string, APCObject> g_objects;
static bool g_initialized = false;

static const std::string packagePath = ros::package::getPath("apc_objects");

APCObject::APCObject()
 : shelfPerceptionMethod(PerceptionMethod::DenseCap)
 , totePerceptionMethod(PerceptionMethod::RBO)
 , topGraspHigh(false)
 , bonusPoints(0)
 , bBox(0,0,0)
 , standingHeight(0)
 , difficulty(0)
 , big(false)
 , erode(0)
 , pick_forbidden(false)
 , stow_forbidden(false)
 , suctionStrength(0.7f)
 , airVelThreshold(0.0f)
 , mass(1.0f)
 , stowGraspOriented(false)
 , registration(false)
{
}

const ObjectMap& objects()
{
	if(!g_initialized)
	{
		for(const std::string& name : object_names)
		{
			APCObject object;
			object.name = name;

			fs::path objectFile = fs::path(packagePath) / "objects" / (name + ".yaml");

			if(fs::exists(objectFile))
			{
				YAML::Node node = YAML::LoadFile(objectFile.string());

				auto grasps = node["heuristic_grasps"];
				if(grasps && grasps.IsSequence())
				{
					for(size_t i = 0; i < grasps.size(); ++i)
					{
						auto grasp = grasps[i].as<std::string>();
						if(grasp == "top")
							object.heuristicGrasps.push_back(APCObject::HeuristicGrasp::Top);
						else if(grasp == "center")
							object.heuristicGrasps.push_back(APCObject::HeuristicGrasp::Center);
						else
						{
							std::stringstream ss;
							ss << "Unknown heuristic grasp type '" << grasp << "' in YAML file '" << objectFile << "'";
							throw std::runtime_error(ss.str());
						}
					}
				}

				auto shelfMethod = node["shelf_perception_method"];
				if(shelfMethod)
				{
					std::string method = shelfMethod.as<std::string>();
					if (method == "DenseCap")
						object.shelfPerceptionMethod = APCObject::PerceptionMethod::DenseCap;
					else if (method == "RBO")
						object.shelfPerceptionMethod = APCObject::PerceptionMethod::RBO;
					else
					{
						std::stringstream ss;
						ss << "Unknown Shelf Perception method '" << shelfMethod << "' in YAML file '" << objectFile << "'";
						throw std::runtime_error(ss.str());
					}
				}

				auto toteMethod = node["tote_perception_method"];
				if(toteMethod)
				{
					std::string method = toteMethod.as<std::string>();
					if (method == "DenseCap")
						object.totePerceptionMethod = APCObject::PerceptionMethod::DenseCap;
					else if (method == "RBO")
						object.totePerceptionMethod = APCObject::PerceptionMethod::RBO;
					else
					{
						std::stringstream ss;
						ss << "Unknown Tote Perception method '" << shelfMethod << "' in YAML file '" << objectFile << "'";
						throw std::runtime_error(ss.str());
					}
				}

				auto topGraspHigh = node["top_grasp_high"];
				if(topGraspHigh)
				{
					object.topGraspHigh = topGraspHigh.as<bool>();
				}

				auto bonusPoints = node["bonus_points"];
				if(bonusPoints)
				{
					object.bonusPoints = bonusPoints.as<int>();
				}

				auto standingHight = node["standing_hight"];
				if(standingHight)
				{
					object.standingHeight = standingHight.as<float>();
				}

				auto boundingBox = node["bBox"];
				if(boundingBox)
				{
					std::vector<float> box = boundingBox.as<std::vector<float> >();
					object.bBox.x() = box.at(0);
					object.bBox.y() = box.at(1);
					object.bBox.z() = box.at(2);
				}

				auto difficulty = node["difficulty"];
				if(difficulty)
				{
					object.difficulty = difficulty.as<int>();
				}

				auto erode = node["erode"];
				if(erode)
					object.erode = erode.as<int>();

				auto pick_forbidden = node["pick_forbidden"];
				if(pick_forbidden)
					object.pick_forbidden = pick_forbidden.as<bool>();

				auto stow_forbidden = node["stow_forbidden"];
				if(stow_forbidden)
					object.stow_forbidden = stow_forbidden.as<bool>();

				auto suctionStrength = node["suction_strength"];
				if(suctionStrength)
					object.suctionStrength = suctionStrength.as<float>();

				auto bigItem= node["big"];
				if(bigItem)
					object.big = bigItem.as<bool>();

				auto mass = node["mass"];
				if(mass)
					object.mass = mass.as<float>();

				auto stowGraspOriented = node["stow_grasp_oriented"];
				if(stowGraspOriented)
					object.stowGraspOriented = stowGraspOriented.as<bool>();

				auto registration = node["registration"];
				if(registration)
					object.registration = registration.as<bool>();

				auto airVelThresh = node["air_velocity_threshold"];
				if(airVelThresh)
					object.airVelThreshold = airVelThresh.as<float>();
			}

			g_objects[name] = object;
		}

		g_initialized = true;
	}

	return g_objects;
}

std::string APCObject::imagePath() const
{
	return packagePath + "/images/" + name + ".png";
}

float apc_objects::APCObject::volume() const
{
	return bBox.x() * bBox.y() * bBox.z();

}


}
