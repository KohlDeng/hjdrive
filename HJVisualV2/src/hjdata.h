#pragma once

#include <unordered_map>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <iostream>

#include <json/json.h>
#include "sqlite3.h"

#include "include/map_data.h"
#include "include/OBFU_subsystem_integrated_types.h"
#include "include/VDYN_subsystem_integrated_types.h"


using namespace std;
using namespace autodrive;

struct StartEndVar
{
    string startVarName;
    string endVarName;
};

struct RawContainer
{
	int bytes;
	vector<char> buffer;
};

struct InfoPkg
{
    uint32 timestamp;
	
	bool hdmap_valid;
	nop::MapData hdmap;
	bool lanes_valid;
	MAP_DataBus lanes;
	bool objs_valid;
	OBJ_DataBus objs;
	bool vdyn_valid;
	VDYN_DataBus vdyn;
};


class HJData
{
public:
	HJData();
	~HJData();
	void parse_json(const char* jsonstr);
	void read_config();
	void read_hcp_stream_10ms();
	void read_hcp_stream_50ms();
	void read_hcp_stream_nopmap();
	void read_camera_lanes(int col);

    void SetPkgs();
	bool GetPkgs(std::vector<InfoPkg> &pkgs);
private:
	string db_file_name;

	Json::Value HCP_Stream_10ms_root;
	Json::Value HCP_Stream_50ms_root;
	Json::Value HCP_Stream_nopmap_root;

	RawContainer raw_container;

	vector<VDYN_DataBus> vdyn;
	std::vector<int> hcp_10ms_timestamps;
	vector<OBJ_DataBus> objs;
	vector<MAP_DataBus> cameraLanes;
	std::vector<uint32_T> objsLatency;
	std::vector<int> hcp_50ms_timestamps;

	vector<nop::MapData> mapdata;
	std::vector<uint32_T> mapLatency;
	//save results
	std::vector<InfoPkg> infopkgs;



};

