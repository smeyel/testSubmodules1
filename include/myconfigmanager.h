#ifndef __MYCONFIGMANAGER_H
#define __MYCONFIGMANAGER_H
#include "stdlib.h"
#include "ConfigManagerBase.h"


class MyConfigManager : public MiscTimeAndConfig::ConfigManagerBase
{
	// This method is called by init of the base class to read the configuration values.
	virtual bool readConfiguration(CSimpleIniA *ini)
	{
		showInputImage = ini->GetBoolValue("Main","showInputImage",false,NULL);
		verboseColorCodedFrame = ini->GetBoolValue("Main","verboseColorCodedFrame",false,NULL);
		interactive = ini->GetBoolValue("Main","interactive",false,NULL);
		logFileName = std::string(ini->GetValue("Main","logFileName","",NULL));
		outputFileName = std::string(ini->GetValue("Main","outputFileName","",NULL));
		cam0FileName = std::string(ini->GetValue("Main","cam0FileName","",NULL));
		cam1FileName = std::string(ini->GetValue("Main","cam1FileName","",NULL));
		cam2FileName = std::string(ini->GetValue("Main","cam2FileName","",NULL));
		camIntrinsicParamsFileName = std::string(ini->GetValue("Main","camIntrinsicParamsFileName","",NULL));
		return true;
	}

public:
	// --- Settings
	bool showInputImage;
	bool verboseColorCodedFrame;
	bool interactive;
	std::string logFileName;
	std::string cam0FileName;
	std::string cam1FileName;
	std::string cam2FileName;
	std::string outputFileName;
	std::string camIntrinsicParamsFileName;
};

#endif

