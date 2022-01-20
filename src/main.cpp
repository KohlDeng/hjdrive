#include <iostream>
#include <string>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>

#include "json/json.h"
#include "sqlite3.h"

#include "common.h"

using namespace std;


const std::unordered_map<string,StartEndVar> varmap
{
    {"OBJ_DataBus",{"HCP_Stream_50ms.Hcp50msStream_Obj.timestamp_high",
					"HCP_Stream_50ms.Hcp50msStream_Obj.padding_2[6]"}},
    {"VDYN_DataBus",{"HCP_Stream_10ms.Hcp10msStream_Vdyn.VDYN_timestamp",
					"HCP_Stream_10ms.Hcp10msStream_Vdyn.VDYN_EstRoadLateralSlopeAngleValid"}},
    {"MapData",{"HCP_Stream_NopMap.HcpNopStream_NopMapData.map_header.timestamp_high32",
				"HCP_Stream_NopMap.HcpNopStream_NopMapData.lanes[3].keypoint.offset"}}
};


HJDrive::HJDrive()
{
	db_file_name = "../dbfile/1201_1548(2).db";
	// db_file_name="/home/dian/Documents/Kohldyh/cpplus/hjdrive/1201_1548(2).db";
}

void HJDrive::parse_json(const char* jsonstr)
{
	Json::Value root;
	Json::Reader reader;

	//cout<<jsonstr<<endl;
	//cout<<sizeof(jsonstr)<<endl;

	if(!reader.parse(jsonstr,jsonstr+strlen(jsonstr),root))
	{
		cout<<"parse json false\n";
		return;
	}

	HCP_Stream_10ms_root=root["HCP_Stream_10ms"];
	HCP_Stream_50ms_root=root["HCP_Stream_50ms"];
	HCP_Stream_nopmap_root=root["HCP_Stream_NopMap"];


	cout<<"type is:"<<HCP_Stream_nopmap_root.type()<<endl;


}

void HJDrive::read_config()
{
	sqlite3 *db;
	sqlite3_stmt *stmt = NULL;
	const char* sql = "SELECT * FROM config";
	int rc;
	int result;

	const char *text;

	rc = sqlite3_open(db_file_name.c_str(),&db);
	if(rc)
	{

		fprintf(stderr,"can't open database: %s\n",sqlite3_errmsg(db));
		return;
	}
	else
	{
		fprintf(stderr,"open database successfully\n");

		result = sqlite3_prepare_v2(db,sql,strlen(sql),&stmt,NULL);
		if(result==SQLITE_OK)
		{
			fprintf(stderr,"sql usage successfully\n");
			//config table 只有 1行
			//cout<<"col count is:"<<sqlite3_column_count(stmt)<<endl;
			sqlite3_step(stmt);
			// cout<<"col id:"<<sqlite3_column_int(stmt,0)<<endl;			//第0列代表ID
			// cout<<"col timestamp:"<<sqlite3_column_int(stmt,1)<<endl;	//第1列代表timestamp
			text =(const char*)sqlite3_column_text(stmt,3);						//第3列是json字符串
			//开始解析json文件
			//cout<<text<<endl;
			parse_json(text);

		}
		else
		{
			fprintf(stderr,"sql usage error\n");
		}
	}	

	sqlite3_finalize(stmt);
	sqlite3_close_v2(db);
	return;
}

void HJDrive::read_hcp_stream_10ms()
{
	sqlite3 *db;
	sqlite3_stmt *stmt = NULL;
	ofstream outfile;
	const char* sql = "SELECT * FROM data_HCP_Stream_10ms";
	int rc,result,rs;
	int start, end,length;		//变量在hcp_stream_10ms里的始末位置
	int i=0;

	string var = "VDYN_DataBus";
	outfile.open("../data.txt");
	const void* blobptr;
	int blobsize;
	// cout<<HCP_Stream_10ms_root.toStyledString()<<endl;	
	// cout<<varmap.at(var).startVarName<<endl;
	// cout<<HCP_Stream_10ms_root[varmap.at(var).startVarName]<<endl;

	//起始点在json数组的第0个位置上，int类型
	start = HCP_Stream_10ms_root[varmap.at(var).startVarName][0].asInt();
	//终止点在json数组的第1个位置上，int类型
	end = HCP_Stream_10ms_root[varmap.at(var).endVarName][1].asInt();
	cout<<"start,end:"<<start<<", "<<end<<endl;
	end = alignData(end);	//四字节对齐
	length = end+1-start;
	cout<<"after aligned, start,end,length:"<<start<<", "<<end<<","<<length<<endl;
	if(start%4!=0) return;

	rc = sqlite3_open(db_file_name.c_str(),&db);
	if(rc){
		fprintf(stderr,"can't open database: %s\n",sqlite3_errmsg(db));
		return;
	}
	else{
		fprintf(stderr,"open database successfully\n");
		result = sqlite3_prepare_v2(db,sql,strlen(sql),&stmt,NULL);
		if(result==SQLITE_OK)
		{
			fprintf(stderr,"sql usage successfully\n");
			cout<<"col count is:"<<sqlite3_column_count(stmt)<<endl;

			sqlite3_step(stmt);
			
			// cout<<"col id:"<<sqlite3_column_int(stmt,0)<<endl;
			// cout<<"col timestamp:"<<sqlite3_column_int64(stmt,1)<<endl;

			blobptr = sqlite3_column_blob(stmt,3);
			blobsize = sqlite3_column_bytes(stmt,3);
			
			// cout<<"size is :"<<blobsize<<endl;

			raw_container.bytes = blobsize;
			raw_container.buffer.resize(blobsize);
			memcpy(raw_container.buffer.data(),blobptr,blobsize);
			// cout<<*raw_container.buffer.begin()<<endl;
			memcpy(&vdyn[0],&raw_container.buffer[0]+start,length);
			outfile<<"vdyn"<<" "<<"timestamp"<<" "<<"yawrate"<<endl;
			outfile<<0<<" ";
			outfile<<vdyn[0].VDYN_timestamp<<" ";
			outfile<<vdyn[0].VDYN_EgoYawRate<<" ";
			outfile<<endl;

	
			// while current
			rs = sqlite3_step(stmt);
			while(rs==SQLITE_ROW)
			{
				i++;
				blobptr = sqlite3_column_blob(stmt,3);
				blobsize = sqlite3_column_bytes(stmt,3);

				raw_container.bytes = blobsize;
				raw_container.buffer.resize(blobsize);
				memcpy(raw_container.buffer.data(),blobptr,blobsize);
				
				memcpy(&vdyn[0],&raw_container.buffer[0]+start,length);

				outfile<<i<<" ";
				outfile<<vdyn[0].VDYN_timestamp<<" ";
				outfile<<vdyn[0].VDYN_EgoYawRate<<" ";
				outfile<<endl;

				rs = sqlite3_step(stmt);
			}


		}
		else{
			fprintf(stderr,"sql usage error\n");
		}
		outfile.close();
		return;	
	}

	sqlite3_finalize(stmt);
	sqlite3_close_v2(db);
	return;

}

int main(int argc,char **argv)
{
	HJDrive hjdrive;
	hjdrive.read_config();
	hjdrive.read_hcp_stream_10ms();
	return 0;
}



