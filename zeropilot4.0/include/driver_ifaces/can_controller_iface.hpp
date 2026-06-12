#pragma once

class ICANController {

protected:
	ICANController() = default;
	
public:
	virtual ~ICANController() = default;
	
	virtual bool routineTasks() = 0;
};
