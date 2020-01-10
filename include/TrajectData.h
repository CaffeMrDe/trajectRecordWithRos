#pragma once
#include <iostream>
#include "vector"
#include "assert.h"
#include "opencv2/highgui.hpp"
#include "dbg.h"
// once six axis data get
/**
 * @brief The TrajectElementData struct
 */
struct TrajectElementData{
    std::vector<double> positionGroup;
    std::vector<double> vectoryGroup;
    std::vector<double> accelerationGroup;
    double timeGroup;
    int id;
};

/**
 * @brief The TrajectGroupData class
 */
class TrajectGroupData{

public:
    TrajectGroupData(){
        id = 0;
    }

public:
    std::vector<double> getPositionElement(int index){

        assert(index >=0 && index <= id);
//        dbg(trajectElementVec[index].positionGroup.size());
        return trajectElementVec[index].positionGroup;
    }

    std::vector<double> getVectoryElement(int index){
        assert(index >=0 && index <= id);
        return trajectElementVec[index].vectoryGroup;
    }

    std::vector<double> getAccelerationElement(int index){
        assert(index >=0 && index <= id);
        return trajectElementVec[index].accelerationGroup;
    }

    double getTimeStampElement(int index){
        assert(index >=0 && index <= id);
        return trajectElementVec[index].timeGroup;
    }

    int size(){
        return id ; //array always bigger than 1 to size
    }


public:
    void setTrajectGroupElementData(std::vector<double> &positionGroup, std::vector<double>& vectoryGroup,  std::vector<double> &accelerationGroup, double timeGroup){
        TrajectElementData  obj;
        obj.positionGroup = positionGroup;
        obj.vectoryGroup = vectoryGroup;
        obj.accelerationGroup = accelerationGroup;
        obj.timeGroup = timeGroup;
        obj.id = this->id;
        trajectElementVec.push_back(obj);

        id++;

    }

private:
    std::vector<TrajectElementData> trajectElementVec;
    int id;

};

/**
 * @brief The TrajectDataIoManager class
 */
class TrajectDataIoManager
{
public:
    TrajectDataIoManager(){}


    TrajectDataIoManager(std::shared_ptr<TrajectGroupData> trajectGroupData) {
        this->trajectGroupData = trajectGroupData;
    }

public:
    void saveData(std::string path="" ){
        createFile(path);

        positionDataFile = cv::FileStorage(path+positionDataFilePath, cv::FileStorage::APPEND);
        vectoryDataFile = cv::FileStorage(path+vectoryDataFilePath, cv::FileStorage::APPEND);
        acceleratioDataFile = cv::FileStorage(path+acceleratioDataFilePath, cv::FileStorage::APPEND);
        timeDataFile = cv::FileStorage(path+timeDataFilePath, cv::FileStorage::APPEND);
        dbg("---------");
        assert(this->trajectGroupData->size() != 0 );
        writeDataToFile(positionDataFile, vectoryDataFile,acceleratioDataFile,timeDataFile);
    }

    bool readData(std::string path=""){
        positionDataFile = cv::FileStorage(path+positionDataFilePath, cv::FileStorage::READ);
        vectoryDataFile = cv::FileStorage(path+vectoryDataFilePath, cv::FileStorage::READ);
        acceleratioDataFile = cv::FileStorage(path+acceleratioDataFilePath, cv::FileStorage::READ);
        timeDataFile = cv::FileStorage(path+timeDataFilePath, cv::FileStorage::READ);
        this->trajectGroupData = std::make_shared<TrajectGroupData>();
        int rtn = readDataFromFile(positionDataFile, vectoryDataFile, acceleratioDataFile, timeDataFile,this->trajectGroupData );

        if(rtn == 0)
            return true;
        else
            return false;
    }

    std::shared_ptr<TrajectGroupData> getDataHandler(){
        return this->trajectGroupData;
    }

private:
    void createFile(std::string path){

        positionDataFile = cv::FileStorage(path+positionDataFilePath, cv::FileStorage::WRITE);
        vectoryDataFile = cv::FileStorage(path+vectoryDataFilePath, cv::FileStorage::WRITE);
        acceleratioDataFile = cv::FileStorage(path+acceleratioDataFilePath, cv::FileStorage::WRITE);
        timeDataFile = cv::FileStorage(path+timeDataFilePath, cv::FileStorage::WRITE);
        positionDataFile.release();
        vectoryDataFile.release();
        acceleratioDataFile.release();
        timeDataFile.release();

    }

    void writeDataToFile(cv::FileStorage &positionFileHandler,cv::FileStorage &vectoryFileHandler,cv::FileStorage &accelerationFileHandler,cv::FileStorage & timeFileHandler){
        // write traject data
        writeElementDataSumToFile(timeFileHandler, this->trajectGroupData->size());
        for(int i = 0; i <this->trajectGroupData->size(); i++){
//            position
//            dbg("writeDataToFile");
            std::vector<double> PData = this->trajectGroupData->getPositionElement(i);
            std::vector<double> VData = this->trajectGroupData->getVectoryElement(i);
            std::vector<double> AData = this->trajectGroupData->getAccelerationElement(i);
//            dbg("get data");
            double TData = this->trajectGroupData->getTimeStampElement(i);
            writeElementDataToFile(positionFileHandler, PData,i);
            writeElementDataToFile(vectoryFileHandler, VData,i);
            writeElementDataToFile(accelerationFileHandler, AData, i);
            writeElementDataToFile(timeFileHandler, TData, i);

        }

        positionDataFile.release();
        vectoryDataFile.release();
        acceleratioDataFile.release();
        timeDataFile.release();

    }

    /**
     * vector and any element always offer
     */
    template<typename T>
    void writeElementDataToFile(cv::FileStorage &fileHandler, T data, int index){
        fileHandler<<namePre+std::to_string(index) << data;
    }

    template<typename T>
    void writeElementDataSumToFile(cv::FileStorage &fileHandler, T data){
        fileHandler<<namePre+std::to_string(-1) << data;
    }

    template<typename T>
    int readDataFromFile(T &positionFileHandler, T &vectoryFileHandler, T &accelerationFileHandler, T &timeFileHandler,std::shared_ptr<TrajectGroupData> traj){
        int sum = readElementDataSumToFile(timeFileHandler) - 1;
        if(sum == -1){
            dbg("read error , please check the data");
            return -1;
        }

        for(int i = 0; i < sum ; i++){
            std::vector<double> PData,VData,AData;
            double time;
            readElementDataFromFIle(positionFileHandler, PData,i);
            readElementDataFromFIle(vectoryFileHandler, VData,i);
            readElementDataFromFIle(accelerationFileHandler, AData,i);
            readElementDataFromFIle(timeFileHandler,time, i);

            traj->setTrajectGroupElementData(PData,VData,AData  ,time);
        }
        dbg("readDataFromFile finsish . size : ", traj->size());
        return 0;
    }

    template<typename T>
    void readElementDataFromFIle(cv::FileStorage &fileHandler, T &data, int index){

        fileHandler[namePre+std::to_string(index)] >> data;
//        dbg(data);
    }

    int readElementDataSumToFile(cv::FileStorage &fileHandler){
        int dataSum = -1;
        fileHandler[namePre+std::to_string(-1)] >> dataSum;
        if(dataSum ==0 ||dataSum < 0)
            return -1;
        return dataSum;
    }



private:
    std::shared_ptr<TrajectGroupData> trajectGroupData;
    cv::FileStorage positionDataFile;
    cv::FileStorage vectoryDataFile;
    cv::FileStorage acceleratioDataFile;
    cv::FileStorage timeDataFile;
    const std::string namePre = "No";
    std::string rootDir;
    const std::string positionDataFilePath = "position.json";
    const std::string vectoryDataFilePath = "vectory.json";
    const std::string acceleratioDataFilePath = "acceleration.json";
    const std::string timeDataFilePath = "time.json";
};
