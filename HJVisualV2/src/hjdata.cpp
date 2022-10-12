#include <iostream>
#include <string>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <cmath>
#include <time.h>

#include <json/json.h>
#include "sqlite3.h"

#include "hjdata.h"

int alignData(int a)
{
    if(a%4!=0)
    {
        int b;
        b = a-(a%4);
        return b+3;
    }
    else
        return a-1;
}

const std::unordered_map<string,StartEndVar> varmap
{
    {"OBJ_DataBus",{"HCP_Stream_50ms.Hcp50msStream_Obj.timestamp_high",
                    "HCP_Stream_50ms.Hcp50msStream_Obj.padding_2[6]"}},
    {"VDYN_DataBus",{"HCP_Stream_10ms.Hcp10msStream_Vdyn.VDYN_timestamp",
                    "HCP_Stream_10ms.Hcp10msStream_Vdyn.VDYN_EstRoadLateralSlopeAngleValid"}},
    {"MapData",{"HCP_Stream_NopMap.HcpNopStream_NopMapData.map_header.timestamp_high32",
                "HCP_Stream_NopMap.HcpNopStream_NopMapData.lanes[3].keypoint.offset"}},
    {"cameraLanes",{"HCP_Stream_50ms.Hcp50msStream_Map.timestamp_high",
                "HCP_Stream_50ms.Hcp50msStream_Map.padding_7[6]"}}
};

HJData::HJData()
{
    db_file_name="/home/dian/Documents/vscode/projects/hjdrive/dbfile/1201_1548(2).db";

    read_config();
    read_hcp_stream_10ms();
    read_hcp_stream_50ms();
    read_hcp_stream_nopmap();
}

HJData::~HJData()
{
    //cout<<"app is over"<<endl;
}


void HJData::parse_json(const char* jsonstr)
{
    Json::Value root;
    Json::Reader reader;
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

void HJData::read_config()
{
    sqlite3 *db;
    sqlite3_stmt *stmt = NULL;
    const char* sql = "SELECT * FROM config";
    int rc,rt;

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
        rt = sqlite3_prepare_v2(db,sql,strlen(sql),&stmt,NULL);
        if(rt==SQLITE_OK)
        {
            fprintf(stderr,"sql usage successfully\n");
            sqlite3_step(stmt);
            text =(const char*)sqlite3_column_text(stmt,3);
            parse_json(text);   ////开始解析json文件
        }
        else{
            fprintf(stderr,"sql usage error\n");
        }
    }

    sqlite3_finalize(stmt);
    sqlite3_close_v2(db);
    return;
}

void HJData::read_hcp_stream_10ms()
{
    sqlite3 *db;
    sqlite3_stmt *stmt = NULL;
    ofstream outfile;
    const char* sql = "SELECT * FROM data_HCP_Stream_10ms";
    int rc,rt,rs;
    int start, end,length;		//变量在hcp_stream_10ms里的始末位置
    int i=0;

    string var = "VDYN_DataBus";
    const void* blobptr;
    int blobsize;

    start = HCP_Stream_10ms_root[varmap.at(var).startVarName][0].asInt();
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
        rt=sqlite3_prepare_v2(db,sql,strlen(sql),&stmt,NULL);
        if(rt==SQLITE_OK)
        {
            fprintf(stderr,"sql usage successfully\n");
            cout<<"col count is:"<<sqlite3_column_count(stmt)<<endl;
            rs = sqlite3_step(stmt);
            while(rs==SQLITE_ROW)
            {
                int timestamp = sqlite3_column_int64(stmt,2);
                blobptr = sqlite3_column_blob(stmt,3);
                blobsize = sqlite3_column_bytes(stmt,3);

                raw_container.bytes = blobsize;
                raw_container.buffer.resize(blobsize);
                memcpy(raw_container.buffer.data(),blobptr,blobsize);
                //copy进入结构体
                vdyn.push_back({0});
                memcpy(&vdyn[i],&raw_container.buffer[start],length);
                hcp_10ms_timestamps.push_back(timestamp);

                rs = sqlite3_step(stmt);
                i++;
            }
            cout<<"vdyn size is:"<<vdyn.size()<<endl;
        }
        else{
            fprintf(stderr,"sql usage error\n");
        }
    }

    sqlite3_finalize(stmt);
    sqlite3_close_v2(db);
    return;
}

void HJData::read_hcp_stream_50ms()
{
    sqlite3 *db;
    sqlite3_stmt *stmt = NULL;
    ofstream outfile;
    const char* sql = "SELECT * FROM data_HCP_Stream_50ms";
    int rc,rt,rs;
    int start,end,length; //变量在hcp_stream_50ms里的始末位置
    int i=0;

    string var = "OBJ_DataBus";
    const void *blobptr;
    int blobsize;

    start = HCP_Stream_50ms_root[varmap.at(var).startVarName][0].asInt();
    end = HCP_Stream_50ms_root[varmap.at(var).endVarName][1].asInt();
    cout<<"start,end:"<<start<<", "<<end<<endl;
    end = alignData(end);	//四字节对齐
    length = end+1-start;
    cout<<"after aligned, start,end,length:"<<start<<", "<<end<<","<<length<<endl;
    if(start%4!=0) return;

    rc=sqlite3_open(db_file_name.c_str(),&db);
    if(rc){
        fprintf(stderr,"can't open database: %s\n",sqlite3_errmsg(db));
        return;
    }
    else
    {
        fprintf(stderr,"open database successfully\n");
        rt = sqlite3_prepare_v2(db,sql,strlen(sql),&stmt,NULL);
        if(rt==SQLITE_OK)
        {
            fprintf(stderr,"sql usage successfully\n");
            // cout<<"col count is:"<<sqlite3_column_count(stmt)<<endl;

            rs = sqlite3_step(stmt);
            while(rs==SQLITE_ROW)
            {
                int timestamp = sqlite3_column_int64(stmt,2);
                blobptr = sqlite3_column_blob(stmt,3);
                blobsize = sqlite3_column_bytes(stmt,3);

                raw_container.bytes = blobsize;
                raw_container.buffer.resize(blobsize);
                memcpy(raw_container.buffer.data(),blobptr,blobsize);
                objs.push_back({0});
                memcpy(&objs[i],&raw_container.buffer[start],length);

                objsLatency.push_back(objs[i].latency);
                hcp_50ms_timestamps.push_back(timestamp);

                read_camera_lanes(i);

                rs=sqlite3_step(stmt);
                i++;
            }
            cout<<"objs size is:"<<objs.size()<<endl;
            cout<<"camera map size is:"<<cameraLanes.size()<<endl;
        }
        else{
            fprintf(stderr,"sql usage error\n");
        }
    }

    sqlite3_finalize(stmt);
    sqlite3_close_v2(db);
    return;

}

void HJData::read_hcp_stream_nopmap()
{
    sqlite3 *db;
    sqlite3_stmt *stmt = NULL;
    ofstream outfile;
    const char* sql = "SELECT * FROM data_HCP_Stream_NopMap";

    int rc,rt,rs;
    int start, end,length;	//变量在hcp_stream_nopmap里的始末位置

    string var = "MapData";
    const void* blobptr;
    int blobsize;
    int i=0;

    start = HCP_Stream_nopmap_root[varmap.at(var).startVarName][0].asInt();
    end = HCP_Stream_nopmap_root[varmap.at(var).endVarName][1].asInt();
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
        rt = sqlite3_prepare_v2(db,sql,strlen(sql),&stmt,NULL);
        if(rt!=SQLITE_OK){
            fprintf(stderr,"sql usage error\n");
        }
        else{
            fprintf(stderr,"sql usage successfully\n");
            // cout<<"col count is:"<<sqlite3_column_count(stmt)<<endl;
            rs=sqlite3_step(stmt);
            while(rs==SQLITE_ROW)
            {
                blobptr = sqlite3_column_blob(stmt,3);
                blobsize = sqlite3_column_bytes(stmt,3);
                raw_container.bytes = blobsize;
                raw_container.buffer.resize(blobsize);
                memcpy(raw_container.buffer.data(),blobptr,blobsize);
                mapdata.push_back({0});
                memcpy(&mapdata[i],&raw_container.buffer[start],length);

                mapLatency.push_back(mapdata[i].map_header.udp_index);
                rs=sqlite3_step(stmt);
                i++;
            }
            cout<<"map size is :"<<mapdata.size()<<endl;
        }
    }

    sqlite3_finalize(stmt);
    sqlite3_close_v2(db);
    return;

}

void HJData::read_camera_lanes(int col)
{
    string var = "cameraLanes";

    int start, end,length;	//变量在hcp_stream_50ms里的始末位置

    start = HCP_Stream_50ms_root[varmap.at(var).startVarName][0].asInt();
    end = HCP_Stream_50ms_root[varmap.at(var).endVarName][1].asInt();
    // cout<<"start,end:"<<start<<", "<<end<<endl;
    end = alignData(end);	//四字节对齐
    length = end+1-start;
    // cout<<"camera Map struct size is:"<<sizeof(MAP_DataBus)<<endl;
    // cout<<"after aligned, start,end,length:"<<start<<", "<<end<<","<<length<<endl;
    if(start%4!=0) return;

    cameraLanes.push_back({0});
    memcpy(&cameraLanes[col],&raw_container.buffer[start],length);

}



void HJData::SetPkgs()
{
    infopkgs.clear();

    unsigned int m=0;
    int size = hcp_50ms_timestamps.size();
    // cout<<"map size is :"<<mapdata.size()<<endl;
    // cout<<"50ms data size is "<<size<<endl;
    // cout<<"objs size is:"<<objs.size()<<endl;
    // cout<<"camera map size is:"<<cameraLanes.size()<<endl;

    for(int i=0;i<size;i++)
    {
        InfoPkg pkg;
        if(m==0)
        {
            while(hcp_50ms_timestamps[m]<hcp_10ms_timestamps[0]) {m++;};
            m=m+1;
        }
        // cout<<"m = "<<m<<endl;
        if(m>hcp_50ms_timestamps.size())
            break;

        int l=0;
        int stamp = hcp_50ms_timestamps[m];
        while(hcp_10ms_timestamps[l]<stamp) {l++;};
        if(l>0 && hcp_10ms_timestamps[l]<stamp+15 && hcp_10ms_timestamps[l]>stamp-15)
        {
            pkg.vdyn_valid = true;
            pkg.vdyn = vdyn[l-1];
        }
        else if(l==0)
        {
            pkg.vdyn_valid = true;
            pkg.vdyn = vdyn[0];
        }
        else{
            std::cout<<"not found"<<std::endl;
            cout<<"pkg size is "<<infopkgs.size()<<endl;
            break;
        }

        pkg.objs_valid = true;
        pkg.objs = objs[m];

        pkg.lanes_valid = true;
        pkg.lanes = cameraLanes[m];

        int k=0;
        while(mapLatency[k]<objsLatency[m]) {k++;};
        if(mapLatency[k]==objsLatency[m])
        {
            pkg.hdmap_valid = true;
            pkg.hdmap = mapdata[k];
        }
        pkg.timestamp = hcp_50ms_timestamps[m];

        m++;
        infopkgs.push_back(pkg);
    }

}

bool HJData::GetPkgs(std::vector<InfoPkg> &pkgs)
{
    pkgs.clear();
    pkgs.assign(infopkgs.begin(),infopkgs.end());

    return true;
}


//clock_t start,end;
//start = clock();
//end = clock();
