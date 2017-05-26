// Database for APC
// Author: Christian Lenz <chrislenz@uni-bonn.de>
#include <apc_control/apc_database.h>
#include <boost/lexical_cast.hpp>


void Shelf::loadTaskFile(std::istream& stream)
{

	YAML::Node doc = YAML::Load(stream);
	if(doc.IsNull())
	{
		ROS_WARN("APC_CONTROL: Could not get YAML Task File");
		return;
	}
	try
	{

		const YAML::Node& bin_contents = doc["bin_contents"];

		for(int i=0;i<4;++i){
			for(int j=0;j<3;++j){
				char t='A' + 3 * i + j;
				std::vector<std::string> content;
				std::string bin="bin_";
				bin += t;
				content = bin_contents[bin].as<std::vector<std::string> >();
				Box box;
				box.name = boost::lexical_cast<std::string>(t);
				box.row = i;
				box.col = j;
				for(size_t k = 0; k < content.size(); ++k){
					Item item(content[k]);
					item.objectData=getItemData(item.name);
					box.items.push_back(item);
				}
				boxes[i][j]=box;
			}
		}

		std::vector<std::string> tote_contents;
		tote_contents= doc["tote_contents"].as<std::vector<std::string> >();
		for(unsigned int i=0;i<tote_contents.size();++i){
			Item item(tote_contents[i]);
			item.objectData=getItemData(item.name);
			tote.push_back(item);
		}

		const YAML::Node& work_order = doc["work_order"];

		for(unsigned int i=0;i<work_order.size();++i){
			std::string bin;
			std::string name;
			bin = work_order[i]["bin"].as<std::string>();
			name = work_order[i]["item"].as<std::string>();

			int pos=(int)bin[4]-'A';
			int row,col;
			row=pos/3;
			col=pos%3;

			WorkItem item(name,row,col,-1,-1,bin.substr(4,1),"tote", getItemData(name));
			workOrder.push_back(item);
		}

		return;
	}
	catch(YAML::Exception& e)
	{
		ROS_WARN("Could not parse YAML: %s", e.what());
		return;
	}


}

void Shelf::writeResultFile(std::ofstream& stream)
{
	stream << "{" << std::endl;
	stream << "\t\"bin_contents\": {" << std::endl;

	for(unsigned int i=0;i<4;i++){
		for(unsigned int j=0;j<3;j++){
			if(boxes[i][j].items.size()<1)
				stream << "\t\t\"bin_" << (char)('A' + 3*i+j) << "\": []";
			else{

				stream << "\t\t\"bin_" << (char)('A' + 3*i+j) << "\": [" << std::endl;
				for(unsigned int k=0;k<boxes[i][j].items.size()-1;k++){
					stream << "\t\t\t\"" << boxes[i][j].items[k].name << "\"," << std::endl;
				}
				stream << "\t\t\t\"" << boxes[i][j].items[boxes[i][j].items.size()-1].name << "\"" << std::endl;
				stream << "\t\t]";
			}

			if(!(i==3 && j==2)) stream << ",";
			stream << std::endl;
		}
	}
	stream << "\t}," << std::endl;
	if(tote.size()==0){
		stream << "\t\"tote_contents\": []" << std::endl;
	}
	else{
		stream << "\t\"tote_contents\": [" << std::endl;
		for(unsigned int k=0;k<tote.size()-1;k++){
			stream << "\t\t\"" << tote[k].name << "\"," << std::endl;
		}
		stream << "\t\t\"" << tote[tote.size()-1].name << "\"" << std::endl;
		stream << "\t]" << std::endl;

	}

	stream << "}" << std::endl;

}


void Shelf::moveItem(WorkItem item)
{
	//Item was in tote
	if(item.location == "tote")
	{
		if(tote.size()<1){
			ROS_ERROR("No items in tote!");
			return;
		}
		if(item.destination == "tote") return;

		for(unsigned int i=0;i<tote.size();i++){
			if(tote[i].name==item.name){
				tote.erase(tote.begin()+i);
				boxes[item.dest_row][item.dest_col].items.push_back(Item(item.name));
				return;
			}
		}
		ROS_ERROR("Could not find Item '%s' in tote.",item.name.c_str());
	}
	//Item was in Shelf
	else
	{
		for(unsigned int k=0;k<boxes[item.loc_row][item.loc_col].items.size();k++){
			if(item.name==boxes[item.loc_row][item.loc_col].items[k].name){
				boxes[item.loc_row][item.loc_col].items.erase(boxes[item.loc_row][item.loc_col].items.begin()+k);
				if(item.destination == "tote")
				{
					tote.push_back(Item(item.name));
				}
				else if(item.dest_col>=0 && item.dest_row >=0)
				{
					boxes[item.dest_row][item.dest_col].items.push_back(Item(item.name));
				}
				else
				{
					ROS_ERROR("Wrong item destination specified for item %s.", item.name.c_str());
				}

				return;
			}
		}
		ROS_ERROR("Could not find Item '%s' in box(%i,%i).",item.name.c_str(),item.loc_row,item.loc_col);
	}
}


void Shelf::removeItem(WorkItem item)
{
	//Item was in tote
	if(item.location == "tote")
	{
		if(tote.size()<1){
			ROS_ERROR("No items in tote!");
			return;
		}
		for(unsigned int i=0;i<tote.size();i++){
			if(tote[i].name==item.name){
				tote.erase(tote.begin()+i);
				return;
			}
		}
		ROS_ERROR("Could not find Item '%s' in tote.",item.name.c_str());
	}

	//Item was in shelf
	else
	{
		for(unsigned int k=0;k<boxes[item.loc_row][item.loc_col].items.size();k++){
			if(item.name==boxes[item.loc_row][item.loc_col].items[k].name){
				boxes[item.loc_row][item.loc_col].items.erase(boxes[item.loc_row][item.loc_col].items.begin()+k);
				return;
			}
		}
		ROS_ERROR("Could not find Item '%s' in box(%i,%i).",item.name.c_str(),item.loc_row,item.loc_col);
	}
}


bool Shelf::checkBoxSize(int row, int col){
	if(row<0 || row >3 || col <0 || col > 2) return false;
	return true;
}

const apc_objects::APCObject* Shelf::getItemData(std::string name)
{
	auto it = apc_objects::objects().find(name);
	if(it == apc_objects::objects().end())
	{
		ROS_ERROR("Specified unknown object '%s'. Can't compute item volume.", name.c_str());
		return new apc_objects::APCObject;
	}
	return &it->second;
}



void Shelf::ItemScoreDifficulty()
{
	for(unsigned int i=0;i<workOrder.size();i++)
	{
		switch(boxes[workOrder[i].loc_row][workOrder[i].loc_col].items.size())
		{
			case 1: case 2:
				workOrder[i].shelfScore = 10;
				break;
			case 3: case 4:
				workOrder[i].shelfScore = 15;
				break;
			case 5: case 6: case 7: case 8:
			case 9: case 10:
				workOrder[i].shelfScore = 20;
				break;
			default: workOrder[i].shelfScore = 0;
		};
	}
}


void Shelf::PointsAndVolume()
{
	for(unsigned int row=0;row<4;row++)
	{
		for(unsigned int col=0;col<3;col++)
		{

			float vol=42;			//x-axis
			if(row%3==0) vol*=23.5; //Height=z-axis 1st and 4th row
			else vol*=19.5; 		//Height=z-axis 2nd and 3th row

			if(col%2==0) vol*=27;   //width=y-axis 1st and 3rd col
			else vol*=30; 			//middle col

			boxes[row][col].predictedVolume=vol;

// 			ROS_INFO("Box Vol: %f.",vol);

			for(auto item: boxes[row][col].items)
			{
				boxes[row][col].predictedVolume -= item.objectData->volume();
				if(item.name == "scotch_bubble_mailer"
						|| item.name == "kleenex_paper_towels"
						|| item.name == "hanes_tube_socks")
					boxes[row][col].containBigItem = true;
			}

			switch(boxes[row][col].items.size())
			{
				case 1: case 2:
					boxes[row][col].stowPoints = 10;
					break;
				case 3: case 4:
					boxes[row][col].stowPoints = 15;
					break;
				case 5: case 6: case 7: case 8:
				case 9: case 10:
					boxes[row][col].stowPoints = 20;
					break;
				default: boxes[row][col].stowPoints = 0;
			};

// 			ROS_WARN("# items: %i. points: %i. Vol: %f",boxes[row][col].items.size(),boxes[row][col].stowPoints,boxes[row][col].predictedVolume);
		}
	}
}



Shelf::Shelf(){}


Box::Box():
stowPoints(0),
predictedVolume(0)
{
}

bool Box::containItem(std::string itemName)
{
	for(auto item: items)
	{
		if(item.name == itemName)
			return true;
	}
	return false;
}



Item::Item(std::string n):
name(n)
{
}


WorkItem::WorkItem(std::string name, int loc_row, int loc_col, int dest_row, int dest_col, std::string location, std::string destination, const apc_objects::APCObject* objectData):
name(name),
location(location),
destination(destination),
loc_row(loc_row),
loc_col(loc_col),
dest_row(dest_row),
dest_col(dest_col),
objectData(objectData)
{}

int WorkItem::getItemScore()
{
	return shelfScore + objectData->bonusPoints;
}















