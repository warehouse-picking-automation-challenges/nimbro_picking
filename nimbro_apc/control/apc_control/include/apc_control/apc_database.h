// Database for APC
// Author: Christian Lenz <chrislenz@uni-bonn.de>

#ifndef APC_DATABASE_H
#define APC_DATABASE_H


#include <yaml-cpp/yaml.h>
#include <ros/console.h>
#include <fstream>

#include <apc_objects/apc_objects.h>

class WorkItem{

public:
	std::string name;
	std::string location;
	std::string destination;
	int loc_row;
	int loc_col;
	int dest_row;
	int dest_col;

	int shelfScore = 0;

	int attempted = 0;
	int missing = 0;

	const apc_objects::APCObject* objectData;

	int getItemScore();

	WorkItem(){};
	WorkItem(std::string name, int loc_row, int loc_col, int dest_row, int dest_col, std::string location, std::string destination, const apc_objects::APCObject* objectData);

};

class Item{

	public:
		std::string name;
		Item(std::string n);
		const apc_objects::APCObject* objectData;
};


class Box{
public:

	std::vector<Item> items;
	int stowPoints;
	float predictedVolume;
	bool containBigItem = false;
	bool containItem(std::string itemName);
	std::string name;
	int row;
	int col;
	Box();

};


class Shelf{

public:
	Box boxes[4][3];
	std::vector<WorkItem> workOrder;
	std::vector<Item> tote;

	Shelf();

	void loadTaskFile(std::istream& stream);
	void writeResultFile(std::ofstream& stream);
	
	void moveItem(WorkItem item);
	void removeItem(WorkItem item);
	
	void ItemScoreDifficulty();
	void PointsAndVolume();

	private:

	bool checkBoxSize(int row, int col);

	const apc_objects::APCObject* getItemData(std::string name);


};


#endif
