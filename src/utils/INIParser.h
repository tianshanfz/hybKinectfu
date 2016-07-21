/*
 * INIParser.h
 *
 *  Created on: May 25, 2016
 *      Author: hanyinbo
 */

#ifndef INIPARSER_H_
#define INIPARSER_H_
#include<map>
#include<vector>
#include<fstream>
#include<iostream>
using namespace std;


class ININode
{
public:
    ININode(string root, string key, string value)
    {
        this->root = root;
        this->key = key;
        this->value = value;
    }
    string root;
    string key;
    string value;
};

class SubNode
{
public:
    void insertElement(string key, string value)
    {
        sub_node.insert(pair<string, string>(key, value));
    }
    map<string, string> sub_node;
};

class INIParser
{
public:
    bool readINI(string path);
    string getValue(string root, string key);
    vector<ININode>::size_type getSize(){return _map_ini.size();}
    vector<ININode>::size_type setValue(string root, string key, string value);
    bool writeINI(string path);
    void clear(){_map_ini.clear();}
private:
    map<string, SubNode> _map_ini;
};

#endif /* INIPARSER_H_ */
