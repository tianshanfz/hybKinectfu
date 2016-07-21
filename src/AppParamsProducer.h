/*
 * AppParamsProducer.h
 *
 *  Created on: May 25, 2016
 *      Author: hanyinbo
 */

#ifndef APPPARAMSPRODUCER_H_
#define APPPARAMSPRODUCER_H_
#include"utils/INIParser.h"
#include"utils/cpu_include.h"
#include"AppParams.h"
class AppParamsProducer {
public:
	AppParamsProducer();
	bool readParams(AppParams* params,string config_filename);
	virtual ~AppParamsProducer();
protected:
	bool readParamsFromConfigFile(AppParams* params);
	INIParser _ini_parser;

};

#endif /* APPPARAMSPRODUCER_H_ */
